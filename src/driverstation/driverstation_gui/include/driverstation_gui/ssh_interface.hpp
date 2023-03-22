#ifndef SSH_INTERFACE_HEADER_INCLUDED
#define SSH_INTERFACE_HEADER_INCLUDED

#include <errno.h>
#include <libssh/libssh.h>
#include <optional>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "read_stream.hpp"

namespace ssh_interface{
	namespace{
		// Included from: https://api.libssh.org/stable/libssh_tutor_guided_tour.html
		//////// -------- Begin Inclusion -------- ////////
		int verify_knownhost(ssh_session session)
		{
		    enum ssh_known_hosts_e state;
		    unsigned char *hash = NULL;
		    ssh_key srv_pubkey = NULL;
		    size_t hlen;
		    char buf[10];
		    char *hexa;
		    char *p;
		    int cmp;
		    int rc;

		    rc = ssh_get_server_publickey(session, &srv_pubkey);
		    if (rc < 0) {
		        return -1;
		    }

		    rc = ssh_get_publickey_hash(srv_pubkey,
		                                SSH_PUBLICKEY_HASH_SHA1,
		                                &hash,
		                                &hlen);
		    ssh_key_free(srv_pubkey);
		    if (rc < 0) {
		        return -1;
		    }

		    state = ssh_session_is_known_server(session);
		    switch (state) {
		        case SSH_KNOWN_HOSTS_OK:
		            /* OK */

		            break;
		        case SSH_KNOWN_HOSTS_CHANGED:
		            fprintf(stderr, "Host key for server changed: it is now:\n");
		            // ssh_print_hexa("Public key hash", hash, hlen);  // Depreciated, replaced by following line
					ssh_print_hash(SSH_PUBLICKEY_HASH_SHA256, hash, hlen);
		            fprintf(stderr, "For security reasons, connection will be stopped\n");
		            ssh_clean_pubkey_hash(&hash);

		            return -1;
		        case SSH_KNOWN_HOSTS_OTHER:
		            fprintf(stderr, "The host key for this server was not found but an other"
		                    "type of key exists.\n");
		            fprintf(stderr, "An attacker might change the default server key to"
		                    "confuse your client into thinking the key does not exist\n");
		            ssh_clean_pubkey_hash(&hash);

		            return -1;
		        case SSH_KNOWN_HOSTS_NOT_FOUND:
		            fprintf(stderr, "Could not find known host file.\n");
		            fprintf(stderr, "If you accept the host key here, the file will be"
		                    "automatically created.\n");

		            /* FALL THROUGH to SSH_SERVER_NOT_KNOWN behavior */
					[[fallthrough]];

		        case SSH_KNOWN_HOSTS_UNKNOWN:
		            hexa = ssh_get_hexa(hash, hlen);
		            fprintf(stderr,"The server is unknown. Do you trust the host key?\n");
		            fprintf(stderr, "Public key hash: %s\n", hexa);
		            ssh_string_free_char(hexa);
		            ssh_clean_pubkey_hash(&hash);
		            p = fgets(buf, sizeof(buf), stdin);
		            if (p == NULL) {
		                return -1;
		            }

		            cmp = strncasecmp(buf, "yes", 3);
		            if (cmp != 0) {
		                return -1;
		            }

		            rc = ssh_session_update_known_hosts(session);
		            if (rc < 0) {
		                fprintf(stderr, "Error %s\n", strerror(errno));
		                return -1;
		            }

		            break;
		        case SSH_KNOWN_HOSTS_ERROR:
		            fprintf(stderr, "Error %s", ssh_get_error(session));
		            ssh_clean_pubkey_hash(&hash);
		            return -1;
		    }

		    ssh_clean_pubkey_hash(&hash);
		    return 0;
		}
		//////// -------- End Inclusion -------- ////////
	}

	class SshInterface : public read_stream::BufferedCharReadStream<80>{
		public:
			enum ConnectionStage{
				DISCONNECTED = 0,
				SESSION_MADE = 1,
				SESSION_CONNECTED = 2,
				CHANNEL_MADE = 3,
				CHANNEL_CONNECTED = 4
			};

			SshInterface(){
				this->current_stage = DISCONNECTED;
			}

			SshInterface(const char *host, const char *username, const char *password){
				this->current_stage = DISCONNECTED;
				this->openChannel(host, username, password);
			}

			~SshInterface(){
				this->closeSession();
			}

			int openChannel(const char* host, const char* username, const char* password){
				switch(this->current_stage){ // TODO: FIXME: use [[fallthrough]]; if this should fallthrough (removes ugly warnings)
					case DISCONNECTED:
					default:
						this->session = ssh_new();
						if(this->session == NULL) break;
						this->current_stage = SESSION_MADE;
					case SESSION_MADE:
						ssh_options_set(this->session, SSH_OPTIONS_HOST, host);
						ssh_options_set(this->session, SSH_OPTIONS_USER, username);
						if(ssh_connect(this->session) != SSH_OK) break;
						this->current_stage = SESSION_CONNECTED;
					case SESSION_CONNECTED:
						if(verify_knownhost(this->session) < 0) break;
						if(ssh_userauth_password(this->session, NULL, password) != SSH_AUTH_SUCCESS) break;
						this->channel = ssh_channel_new(this->session);
						if(this->channel == NULL) break;
						this->current_stage = CHANNEL_MADE;
					case CHANNEL_MADE:
						if(ssh_channel_open_session(this->channel) != SSH_OK) break;
						if(ssh_channel_request_pty(this->channel) != SSH_OK) break;
						// TODO: Parameterized terminal size?
						if(ssh_channel_change_pty_size(this->channel, 80, 24) != SSH_OK) break;
						if(ssh_channel_request_shell(this->channel) != SSH_OK) break;
						this->current_stage = CHANNEL_CONNECTED;
					case CHANNEL_CONNECTED:
						return SSH_OK;
				}
				this->closeSession();
				return SSH_ERROR;
			}

			void closeChannel(){
				switch(this->current_stage){ // TODO: FIXME: use [[fallthrough]]; if this should fallthrough (removes ugly warnings)
					case CHANNEL_CONNECTED:
						ssh_channel_close(this->channel);
						ssh_channel_send_eof(this->channel);
					case CHANNEL_MADE:
						ssh_channel_free(this->channel);
						this->current_stage = SESSION_CONNECTED;
					case SESSION_CONNECTED:
					case SESSION_MADE:
					case DISCONNECTED:
					default:
						break;
				}
			}

			void closeSession(){
				switch(this->current_stage){ // TODO: FIXME: use [[fallthrough]]; if this should fallthrough (removes ugly warnings)
					case CHANNEL_CONNECTED:
					case CHANNEL_MADE:
						this->closeChannel();
					case SESSION_CONNECTED:
						ssh_disconnect(this->session);
					case SESSION_MADE:
						ssh_free(this->session);
						this->current_stage = DISCONNECTED;
					case DISCONNECTED:
					default:
						break;
				}
			}

			int readChannel(void* destination_buffer, uint32_t maximum_byte_count){
				int bytes_received = 0;
				if(this->current_stage >= CHANNEL_CONNECTED && ssh_channel_is_open(this->channel) && !ssh_channel_is_eof(this->channel)){
					bytes_received = ssh_channel_read(this->channel, destination_buffer, maximum_byte_count, 0);
					if(bytes_received < 0) return SSH_ERROR;
				}
				return bytes_received;
			}

			// TODO: replace `uint32_t` with 'usize' type?
			int readChannelNonblocking(void* destination_buffer, uint32_t maximum_byte_count){
				int bytes_received = 0;
				if(this->current_stage >= CHANNEL_CONNECTED && ssh_channel_is_open(this->channel) && !ssh_channel_is_eof(this->channel)){
					bytes_received = ssh_channel_read_nonblocking(this->channel, destination_buffer, maximum_byte_count, 0);
					if(bytes_received < 0) return SSH_ERROR;
				}
				return bytes_received;
			}

			int writeChannel(const void* source_buffer, uint32_t byte_count){
				int bytes_sent = 0;
				if(this->current_stage >= CHANNEL_CONNECTED && ssh_channel_is_open(this->channel) && !ssh_channel_is_eof(this->channel)){
					bytes_sent = ssh_channel_write(this->channel, source_buffer, byte_count);
					if(bytes_sent < 0) return SSH_ERROR;
				}
				return bytes_sent;
			}

			ConnectionStage currentStage() const{return this->current_stage;}

		protected:
			ConnectionStage current_stage;
			ssh_session session;
			ssh_channel channel;

			size_t refreshBuffer(char* buffer, size_t capacity) override{
				return this->readChannelNonblocking(buffer, capacity);
			}
	};
}

#endif

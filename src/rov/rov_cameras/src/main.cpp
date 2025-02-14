#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <filesystem>

#define DEFAULT_RTSP_PORT "8554"

static char *port = (char *) DEFAULT_RTSP_PORT;

// Define a structure to hold stream information
typedef struct {
    GstRTSPMediaFactory *factory;
    gchar *mount_point;
} StreamInfo;

static GList *streams = NULL;  // List of active streams

static GOptionEntry entries[] = {
  {"port", 'p', 0, G_OPTION_ARG_STRING, &port,
      "Port to listen on (default: " DEFAULT_RTSP_PORT ")", "PORT"},
  {NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, NULL}
};

GstRTSPServer *server = NULL;
GstRTSPMountPoints *mounts = NULL;

/* Function to add a new RTSP stream */
void add_stream(const gchar *mount_point, const gchar *launch_pipeline) {
    StreamInfo *stream_info = g_new0(StreamInfo, 1);
    
    stream_info->mount_point = g_strdup(mount_point);
    stream_info->factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(stream_info->factory, launch_pipeline);
    gst_rtsp_media_factory_set_shared(stream_info->factory, TRUE);

    gst_rtsp_mount_points_add_factory(mounts, stream_info->mount_point, stream_info->factory);
    streams = g_list_append(streams, stream_info);

    g_print("Added stream at rtsp://127.0.0.1:%s%s\n", port, mount_point);
}

/* Function to remove an RTSP stream */
void remove_stream(const gchar *mount_point) {
    GList *iter = streams;
    while (iter != NULL) {
        StreamInfo *stream_info = (StreamInfo *) iter->data;
        if (g_strcmp0(stream_info->mount_point, mount_point) == 0) {
            gst_rtsp_mount_points_remove_factory(mounts, mount_point);
            g_free(stream_info->mount_point);
            g_clear_object(&stream_info->factory);
            streams = g_list_remove(streams, stream_info);
            g_free(stream_info);
            g_print("Removed stream at %s\n", mount_point);
            return;
        }
        iter = iter->next;
    }
    g_print("Stream at %s not found\n", mount_point);
}

/* Function to restart an RTSP stream */
void restart_stream(const gchar *mount_point, const gchar *new_pipeline) {
    remove_stream(mount_point);
    add_stream(mount_point, new_pipeline);
}

int main (int argc, char *argv[]) {
    GMainLoop *loop;
    GOptionContext *optctx;
    GError *error = NULL;

    // Parse options
    optctx = g_option_context_new ("- RTSP Server Example with Multiple Streams");
    g_option_context_add_main_entries (optctx, entries, NULL);
    g_option_context_add_group (optctx, gst_init_get_option_group ());
    if (!g_option_context_parse (optctx, &argc, &argv, &error)) {
        g_printerr ("Error parsing options: %s\n", error->message);
        g_option_context_free (optctx);
        g_clear_error (&error);
        return -1;
    }
    g_option_context_free (optctx);

    // Create the main loop
    loop = g_main_loop_new (NULL, FALSE);

    // Create a new RTSP server instance
    server = gst_rtsp_server_new();
    g_object_set(server, "service", port, NULL);

    // Get the mount points for the server
    mounts = gst_rtsp_server_get_mount_points(server);

    // Start with the default streams (front and transect cameras)
    add_stream("/rov_front", "( nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96 )");
    add_stream("/rov_transect", "( nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96 )");

    // Attach the server to the default main context
    gst_rtsp_server_attach(server, NULL);

    g_print("RTSP Server ready at rtsp://127.0.0.1:%s/rov_front\n", port);
    g_print("RTSP Server ready at rtsp://127.0.0.1:%s/rov_transect\n", port);
    g_main_loop_run(loop);

    // Cleanup
    g_list_free_full(streams, g_free);
    g_object_unref(mounts);
    g_object_unref(server);
    g_main_loop_unref(loop);

    return 0;
}


#include <gst/gst.h>
#include <iostream>

#define PIPELINE_DUMMY ""
#define PIPELINE_FRONT ""
#define PIPELINE_TRANSECT "\"nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96\""

#define FRONT_CAMERA "/dev/rov_front"
#define TRANSECT_CAMERA "/dev/rov_transect"

// TODO: Check that this is how the kernel assigns camera ids to the MIPI ports
// TODO: Check that this is still how the kernel works when USB cameras are present
#define FRONT_CAMERA_ARGUS_SENSOR_ID 0
#define TRANSECT_CAMERA_ARGUS_SENSOR_ID 1

int main(int argc, char *argv[]) {
    // Initialize GStreamer
    gst_init(&argc, &argv);
`
    // Define the device (default: /dev/video0)
    const char *device = "/dev/video0";

    // Create a pipeline using gst_parse_launch
    GstElement *pipeline = gst_parse_launch(
        ("v4l2src device=" + std::string(device) + " ! videoconvert ! autovideosink").c_str(),
        nullptr);

    if (!pipeline) {
        std::cerr << "Failed to create pipeline." << std::endl;
        return -1;
    }

    // Set the pipeline to the PLAYING state
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Unable to set the pipeline to the playing state." << std::endl;
        gst_object_unref(pipeline);an example of settin
        return -1;
    }

    // Wait until error or EOS
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                                 static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    // Handle the message
    if (msg != nullptr) {
        GError *err;
        gchar *debug_info;

        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug_info);
                std::cerr << "Error received from element " << GST_OBJECT_NAME(msg->src)
                          << ": " << err->message << std::endl;
                std::cerr << "Debugging information: " << (debug_info ? debug_info : "none") << std::endl;
                g_clear_error(&err);
                g_free(debug_info);
                break;
            case GST_MESSAGE_EOS:
                std::cout << "End-Of-Stream reached." << std::endl;
                break;
            default:
                std::cerr << "Unexpected message received." << std::endl;
                break;
        }
        gst_message_unref(msg);
    }

    // Free resources
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}
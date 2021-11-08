#include <arv.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

image_transport::Publisher publisher;
ros::NodeHandle *node_handle;
ros::Time p_cb_time;

gint width;
gint height;

typedef struct {
	GMainLoop *main_loop;
	int buffer_count;
} ApplicationData;

static gboolean cancel = FALSE;

static void
set_cancel (int signal)
{
	cancel = TRUE;
}

static void
new_buffer_cb (ArvStream *stream, ApplicationData *data)
{
	ArvBuffer *buffer;

	buffer = arv_stream_try_pop_buffer (stream);
	if (buffer != NULL) {
		if (arv_buffer_get_status (buffer) == ARV_BUFFER_STATUS_SUCCESS)
			data->buffer_count++;
		/* Image processing here */

        int step = width; // XXX how to check this?

        size_t buffer_size;
        const uint8_t * buffer_data = static_cast<const uint8_t *>(arv_buffer_get_data(buffer, &buffer_size));

        std::vector<uint8_t> this_data(buffer_size);
        memcpy(&this_data[0], buffer_data, buffer_size);

        sensor_msgs::Image msg;
        msg.header.stamp = ros::Time::now(); // host timestamps (else buffer->timestamp_ns)
        msg.header.seq = arv_buffer_get_frame_id (buffer); msg.header.frame_id = "camera";
        msg.height = height;
        msg.width = width;
        msg.encoding = "mono8";  // XXX fixme
        msg.step = step;
        msg.data = this_data;

        publisher.publish(msg);
    
		arv_stream_push_buffer (stream, buffer);
	}
}

static gboolean
periodic_task_cb (void *abstract_data)
{
	ApplicationData *data = (ApplicationData*)abstract_data;

	printf ("Frame rate = %d Hz\n", data->buffer_count);
	data->buffer_count = 0;

	if (cancel) {
		g_main_loop_quit (data->main_loop);
		return FALSE;
	}

	return TRUE;
}

static void
control_lost_cb (ArvGvDevice *gv_device)
{
	/* Control of the device is lost. Display a message and force application exit */
	printf ("Control lost\n");

	cancel = TRUE;
}

int
main (int argc, char **argv)
{

  ros::init(argc, argv, "aravis");
  node_handle = new ros::NodeHandle();

  image_transport::ImageTransport *transport = new image_transport::ImageTransport(*node_handle);
  publisher = transport->advertise("image_raw", 1);

	ApplicationData data;
	ArvCamera *camera;
	ArvStream *stream;
	GError *error = NULL;
	int i;

	data.buffer_count = 0;

	/* Instantiation of the first available camera */
	camera = arv_camera_new (NULL, &error);

	if (ARV_IS_CAMERA (camera)) {
		void (*old_sigint_handler)(int);
		gint payload;

		/* Set region of interrest to a 200x200 pixel area */
		//arv_camera_set_region (camera, 0, 0, 200, 200, NULL);
    gint x,y;
    arv_camera_get_region (camera, &x, &y, &width, &height, NULL);
		/* Set frame rate to 10 Hz */
		arv_camera_set_frame_rate (camera, 10.0, NULL);
		/* retrieve image payload (number of bytes per image) */
		payload = arv_camera_get_payload (camera, NULL);

		/* Create a new stream object */
		stream = arv_camera_create_stream (camera, NULL, NULL, &error);

		if (ARV_IS_STREAM (stream)) {
			/* Push 50 buffer in the stream input buffer queue */
			for (i = 0; i < 50; i++)
				arv_stream_push_buffer (stream, arv_buffer_new (payload, NULL));

			/* Start the video stream */
			arv_camera_start_acquisition (camera, NULL);

			/* Connect the new-buffer signal */
			g_signal_connect (stream, "new-buffer", G_CALLBACK (new_buffer_cb), &data);
			/* And enable emission of this signal (it's disabled by default for performance reason) */
			arv_stream_set_emit_signals (stream, TRUE);

			/* Connect the control-lost signal */
			g_signal_connect (arv_camera_get_device (camera), "control-lost",
					  G_CALLBACK (control_lost_cb), NULL);

			/* Install the callback for frame rate display */
			g_timeout_add_seconds (1, periodic_task_cb, &data);

			/* Create a new glib main loop */
			data.main_loop = g_main_loop_new (NULL, FALSE);

			old_sigint_handler = signal (SIGINT, set_cancel);

			/* Run the main loop */
			g_main_loop_run (data.main_loop);

			signal (SIGINT, old_sigint_handler);

			g_main_loop_unref (data.main_loop);

			/* Stop the video stream */
			arv_camera_stop_acquisition (camera, NULL);

			/* Signal must be inhibited to avoid stream thread running after the last unref */
			arv_stream_set_emit_signals (stream, FALSE);

			g_object_unref (stream);
		} else {
			printf ("Can't create stream thread%s%s\n",
				error != NULL ? ": " : "",
				error != NULL ? error->message : "");

			g_clear_error (&error);
		}

		g_object_unref (camera);
	} else {
		printf ("No camera found%s%s\n",
			error != NULL ? ": " : "",
			error != NULL ? error->message : "");
		g_clear_error (&error);
	}

	return 0;
}


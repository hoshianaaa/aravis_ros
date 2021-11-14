#include <arv.h>
#include <stdlib.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;


int
main (int argc, char **argv)
{
  std::cout << "version: " << CV_VERSION << std::endl;

	ArvCamera *camera;
	ArvBuffer *buffer;

	camera = arv_camera_new (argc > 1 ? argv[1] : NULL, NULL);
	buffer = arv_camera_acquisition (camera, 0, NULL);

	if (ARV_IS_BUFFER (buffer) && arv_buffer_get_status (buffer) == ARV_BUFFER_STATUS_SUCCESS)
  {
		printf ("Image successfully acquired\n");
    printf (" Converting Image to MAT\n");

  }
	else
		printf ("Failed to acquire a single image\n");

	g_clear_object (&camera);
	g_clear_object (&buffer);

	return EXIT_SUCCESS;
}

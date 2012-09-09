/*******************************************************************************
#	 	luvcopencv: OpenCV video Usb Video Class grabber                       #
# This package is based on the luvcview package by                             #
# Laurent Pinchart &&  Michel Xhaard                                           #
#.                                                                             #
# Copyright (C) 2012 Markus Bader                                              #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>

extern "C" {
#include "luvcview/v4l2uvc.h"
#include "luvcview/gui.h"
#include "luvcview/utils.h"
#include "luvcview/color.h"
}

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
/* Fixed point arithmetic */
#define FIXED Sint32
#define FIXED_BITS 16
#define TO_FIXED(X) (((Sint32)(X))<<(FIXED_BITS))
#define FROM_FIXED(X) (((Sint32)(X))>>(FIXED_BITS))

#define INCPANTILT 64 // 1°

static const char version[] = VERSION;
struct vdIn *videoIn;

/* translate YUV422Packed to bgr24 */

void Pyuv422tobgr24(unsigned char *input_ptr, unsigned char *output_ptr, unsigned int image_width, unsigned int image_height)
{
    unsigned int i, size;
    unsigned char Y, Y1, U, V;
    unsigned char *buff = input_ptr;
    unsigned char *output_pt = output_ptr;
    size = image_width * image_height / 2;
    for(i = size; i > 0; i--) {
        /* bgr instead rgb ?? */
        Y = buff[0] ;
        U = buff[1] ;
        Y1 = buff[2];
        V = buff[3];
        buff += 4;
        *output_pt++ = B_FROMYU(Y, U);
        *output_pt++ = G_FROMYUV(Y, U, V);
        *output_pt++ = R_FROMYV(Y, V);

        *output_pt++ = B_FROMYU(Y1, U);
        *output_pt++ = G_FROMYUV(Y1, U, V);
        *output_pt++ = R_FROMYV(Y1, V);
    }
}

int main(int argc, char *argv[])
{
    int hwaccel = 0;
    const char *videodevice = NULL;
    const char *mode = NULL;
    int format = V4L2_PIX_FMT_MJPEG;
    int i;
    int grabmethod = 1;
    int width = 640;
    int height = 480;
    float fps = 30.0;			// Requested frame rate
    float frmrate = 0.0;		// Measured frame rate
    char *avifilename = NULL;
    int queryformats = 0;
    int querycontrols = 0;
    int readconfigfile = 0;
    char *separateur;
    char *sizestring = NULL;
    char *fpsstring  = NULL;
    int enableRawStreamCapture = 0;
    int enableRawFrameCapture = 0;

    printf("luvcview %s\n\n", version);
    for(i = 1; i < argc; i++) {
        /* skip bad arguments */
        if(argv[i] == NULL || *argv[i] == 0 || *argv[i] != '-') {
            continue;
        }
        if(strcmp(argv[i], "-d") == 0) {
            if(i + 1 >= argc) {
                printf("No parameter specified with -d, aborting.\n");
                exit(1);
            }
            videodevice = strdup(argv[i + 1]);
        }
        if(strcmp(argv[i], "-g") == 0) {
            /* Ask for read instead default  mmap */
            grabmethod = 0;
        }
        if(strcmp(argv[i], "-w") == 0) {
            /* disable hw acceleration */
            hwaccel = 1;
        }
        if(strcmp(argv[i], "-f") == 0) {
            if(i + 1 >= argc) {
                printf("No parameter specified with -f, aborting.\n");
                exit(1);
            }
            mode = argv[i + 1];

            if(strcasecmp(mode, "yuv") == 0 || strcasecmp(mode, "YUYV") == 0) {
                format = V4L2_PIX_FMT_YUYV;
            } else if(strcasecmp(mode, "uyvy") == 0 || strcasecmp(mode, "UYVY") == 0) {
                format = V4L2_PIX_FMT_UYVY;
            } else if(strcasecmp(mode, "jpg") == 0 || strcasecmp(mode, "MJPG") == 0) {
                format = V4L2_PIX_FMT_MJPEG;
            } else {
                printf("Unknown format specified. Aborting.\n");
                exit(1);
            }
        }
        if(strcmp(argv[i], "-s") == 0) {
            if(i + 1 >= argc) {
                printf("No parameter specified with -s, aborting.\n");
                exit(1);
            }

            sizestring = strdup(argv[i + 1]);

            width = strtoul(sizestring, &separateur, 10);
            if(*separateur != 'x') {
                printf("Error in size use -s widthxheight\n");
                exit(1);
            } else {
                ++separateur;
                height = strtoul(separateur, &separateur, 10);
                if(*separateur != 0)
                    printf("hmm.. dont like that!! trying this height\n");
            }
        }
        if(strcmp(argv[i], "-i") == 0) {
            if(i + 1 >= argc) {
                printf("No parameter specified with -i, aborting.\n");
                exit(1);
            }
            fpsstring = argv[i + 1];
            fps = strtof(fpsstring, &separateur);
            if(*separateur != '\0') {
                printf("Invalid frame rate '%s' specified with -i. "
                       "Only numbers are supported. Aborting.\n", fpsstring);
                exit(1);
            }
        }
        if(strcmp(argv[i], "-S") == 0) {
            /* Enable raw stream capture from the start */
            enableRawStreamCapture = 1;
        }
        if(strcmp(argv[i], "-c") == 0) {
            /* Enable raw frame capture for the first frame */
            enableRawFrameCapture = 1;
        }
        if(strcmp(argv[i], "-C") == 0) {
            /* Enable raw frame stream capture from the start*/
            enableRawFrameCapture = 2;
        }
        if(strcmp(argv[i], "-o") == 0) {
            /* set the avi filename */
            if(i + 1 >= argc) {
                printf("No parameter specified with -o, aborting.\n");
                exit(1);
            }
            avifilename = strdup(argv[i + 1]);
        }
        if(strcmp(argv[i], "-L") == 0) {
            /* query list of valid video formats */
            queryformats = 1;
        }
        if(strcmp(argv[i], "-l") == 0) {
            /* query list of valid video formats */
            querycontrols = 1;
        }

        if(strcmp(argv[i], "-r") == 0) {
            /* query list of valid video formats */
            readconfigfile = 1;
        }
        if(strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("usage: uvcview [-h -d -g -f -s -i -c -o -C -S -L -l -r]\n");
            printf("-h   print this message\n");
            printf("-d   /dev/videoX       use videoX device\n");
            printf("-g   use read method for grab instead mmap\n");
            printf("-w   disable SDL hardware accel.\n");
            printf("-f   choose video format (YUYV/yuv, UYVY/uyvy and MJPG/jpg are valid, MJPG is default)\n");
            printf("-i   fps           use specified frame rate\n");
            printf("-s   widthxheight      use specified input size\n");
            printf("-c   enable raw frame capturing for the first frame\n");
            printf("-C   enable raw frame stream capturing from the start\n");
            printf("-S   enable raw stream capturing from the start\n");
            printf("-o   avifile  create avifile, default video.avi\n");
            printf("-L   query valid video formats\n");
            printf("-l   query valid controls and settings\n");
            printf("-r   read and set control settings from luvcview.cfg (save/restore with F1/F2)\n");
            exit(0);
        }
    }
    if(videodevice == NULL || *videodevice == 0) {
        videodevice = "/dev/video0";
    }

    if(avifilename == NULL || *avifilename == 0) {
        avifilename = (char *) "video.avi";
    }

    videoIn = (struct vdIn *) calloc(1, sizeof(struct vdIn));
    if(queryformats) {
        /* if we're supposed to list the video formats, do that now and go out */
        check_videoIn(videoIn, (char *) videodevice);
        free(videoIn);
        exit(1);
    }
    if(init_videoIn
            (videoIn, (char *) videodevice, width, height, fps, format,
             grabmethod, avifilename) < 0)
        exit(1);
    /* if we're supposed to list the controls, do that now */
    if(querycontrols)
        enum_controls(videoIn->fd);

    /* if we're supposed to read the control settings from a configfile, do that now */
    if(readconfigfile)
        load_controls(videoIn->fd);

    if(enableRawStreamCapture) {
        videoIn->captureFile = fopen("stream.raw", "wb");
        if(videoIn->captureFile == NULL) {
            perror("Unable to open file for raw stream capturing");
        } else {
            printf("Starting raw stream capturing to stream.raw ...\n");
        }
    }
    if(enableRawFrameCapture)
        videoIn->rawFrameCapture = enableRawFrameCapture;
    initLut();

    // Initialize frame rate calculator
    double elapsedTime;
    timeval nowtime, lasttime;
    gettimeofday(&nowtime, NULL);
    lasttime = nowtime;
    /* main big loop */
    cv::Mat img(videoIn->height, videoIn->width, CV_8UC3);
    while(videoIn->signalquit) {
        // Measure the frame rate every (fps/2) frames
        gettimeofday(&nowtime, NULL);
        elapsedTime = (nowtime.tv_sec - lasttime.tv_sec) * 1000.0;      // sec to ms
        elapsedTime += (nowtime.tv_usec - lasttime.tv_usec) / 1000.0;   // us to ms
        lasttime = nowtime;
        frmrate = 1000.0 / elapsedTime;

        if(uvcGrab(videoIn) < 0) {
            printf("Error grabbing\n");
            break;
        }

        /* if we're grabbing video, show the frame rate */
        if(videoIn->toggleAvi)
            printf("\rframe rate: %g     ", frmrate);

        if(videoIn->getPict) {
            switch(videoIn->formatIn) {
            case V4L2_PIX_FMT_MJPEG:
                get_picture(videoIn->tmpbuffer, videoIn->buf.bytesused);
                break;
            case V4L2_PIX_FMT_YUYV:
                get_pictureYV2(videoIn->framebuffer, videoIn->width, videoIn->height);
                break;
            default:
                break;
            }
            videoIn->getPict = 0;
            printf("get picture !\n");
        }

        Pyuv422tobgr24(videoIn->framebuffer, img.data, videoIn->width, videoIn->height);
        cv::imshow("Cam", img);
        cv::waitKey(2);

    }

    /* if avifile is defined, we made a video: compute the exact fps and
       set it in the video */
    if(videoIn->avifile != NULL) {
        float fps = (videoIn->framecount / (videoIn->recordtime / 1000));
        fprintf(stderr, "setting fps to %f\n", fps);
        AVI_set_video(videoIn->avifile, videoIn->width, videoIn->height,
                      fps, (char *) "MJPG");
        AVI_close(videoIn->avifile);
    }

    close_v4l2(videoIn);
    free(videoIn);
    freeLut();
    printf("Cleanup done. Exiting ...\n");
}

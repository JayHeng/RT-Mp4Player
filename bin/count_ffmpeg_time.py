import sys, os
import argparse
import numpy as np
import matplotlib.pyplot as plt

def GetMax(lst):
   return ("%.3f" % max(lst))

def GetMin(lst):
   return ("%.3f" % min(lst))

def GetMedian(lst):
   data = sorted(lst)
   size = len(lst)
   if size % 2 == 0:
      median = (data[size//2]+data[size//2-1])/2
   if size % 2 == 1:
      median = data[(size-1)//2]
   return ("%.3f" % median)

def GetAverage(lst):
   sum = 0.0
   for item in lst:
      sum += item
   return ("%.3f" % (sum/len(lst)))

class ShowFFmpegTime(object):
    def __init__(self):
        pass

    def _read_options(self):
        parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter)
        parser.add_argument("input", help="FFmpeg decode time data file."),
        parser.add_argument("vinfo", help="Video resolution info."),
        parser.add_argument("build", help="Project build info."),
        return parser.parse_args()

    def _process(self):
        self.readFrameList = []
        self.decodeAudioList = []
        self.decodeVideoList = []
        self.pxpConvVideoList = []
        self.lcdDispVideoList = []

        with open(self._inputFile, 'r') as fileObj:
            try:
                while(True):
                    # one line frame data like this:
                    #        read            decode audio       decode video      PXP conv video     LCD show video
                    # '0x0000000000006000,0x0000000000000000,0x0000100000006000,0x0000200000006000,0x0000200000006000,'
                    frameData = fileObj.readline()
                    if frameData:
                        readFrameIndex = frameData.find(',')
                        if readFrameIndex != -1:
                            readFrameTime = int(frameData[2:readFrameIndex], 16)
                            self.readFrameList.extend([readFrameTime / 1000.0])

                        decodeAudioIndex = frameData.find(',', readFrameIndex + 1)
                        if decodeAudioIndex != -1:
                            decodeAudioTime = int(frameData[readFrameIndex + 3:decodeAudioIndex], 16)
                            if decodeAudioTime:
                                self.decodeAudioList.extend([decodeAudioTime / 1000000.0])

                        decodeVideoIndex = frameData.find(',', decodeAudioIndex + 1)
                        if decodeVideoIndex != -1:
                            decodeVideoTime = int(frameData[decodeAudioIndex + 3:decodeVideoIndex], 16)
                            if decodeVideoTime:
                                self.decodeVideoList.extend([decodeVideoTime / 1000000.0])

                        pxpConvVideoIndex = frameData.find(',', decodeVideoIndex + 1)
                        if pxpConvVideoIndex != -1:
                            pxpConvVideoTime = int(frameData[decodeVideoIndex + 3:pxpConvVideoIndex], 16)
                            if pxpConvVideoTime:
                                self.pxpConvVideoList.extend([pxpConvVideoTime / 1000000.0])

                        lcdDispVideoIndex = frameData.find(',', pxpConvVideoIndex + 1)
                        if lcdDispVideoIndex != -1:
                            lcdDispVideoTime = int(frameData[pxpConvVideoIndex + 3:lcdDispVideoIndex], 16)
                            if lcdDispVideoTime:
                                self.lcdDispVideoList.extend([lcdDispVideoTime / 1000000.0])
                    else:
                        break
            finally:
                fileObj.close()

    def _finalize(self):
        print ("Processing is completed!")

    def _show(self):
        dt = 1
        fig, axs = plt.subplots(3, 2)
        axs[0][0].set_title(self._projBuild, color='b')
        #axs[0].set_title('Text & Data & Heap & Stack & frameBuffer in SDRAM', color='b')
        #axs[0].set_title('Text in FlexRAM,  Data & Heap & Stack & frameBuffer in SDRAM', color='b')
        #axs[0].set_title('Text in Hyper, Data & Heap & Stack & frameBuffer in SDRAM', color='b')
        #axs[0].set_title('Text in Hyper, Data in FlexRAM, Heap & Stack & frameBuffer in SDRAM', color='b')

        if len(self.readFrameList):
            readFrames = np.arange(0, len(self.readFrameList), dt)
            statistics0 = 'Max=' + GetMax(self.readFrameList) + \
                        '\nMin=' + GetMin(self.readFrameList) + \
                        '\nMedian=' + GetMedian(self.readFrameList) + \
                        '\nAverage=' + GetAverage(self.readFrameList)
            axs[0][0].plot(readFrames, self.readFrameList, label=statistics0)
            vinfo = 'media frame(' + self._videoInfo  + ') index \n'
            axs[0][0].legend(loc='upper right', fontsize=6, shadow=True, fancybox=True)
            axs[0][0].set_xlabel(vinfo)
            axs[0][0].set_ylabel('read time (us)')
            axs[0][0].grid(True)

        if len(self.decodeAudioList):
            audioFrames = np.arange(0, len(self.decodeAudioList), dt)
            statistics1 = 'Max=' + GetMax(self.decodeAudioList) + \
                        '\nMin=' + GetMin(self.decodeAudioList) + \
                        '\nMedian=' + GetMedian(self.decodeAudioList) + \
                        '\nAverage=' + GetAverage(self.decodeAudioList)
            axs[1][0].plot(audioFrames, self.decodeAudioList, label=statistics1)
            axs[1][0].legend(loc='upper right', fontsize=6, shadow=True, fancybox=True)
            axs[1][0].set_xlabel('audio frame index')
            axs[1][0].set_ylabel('decode time (ms)')
            axs[1][0].grid(True)

        if len(self.decodeVideoList):
            dVideoFrames = np.arange(0, len(self.decodeVideoList), dt)
            statistics2 = 'Max=' + GetMax(self.decodeVideoList) + \
                        '\nMin=' + GetMin(self.decodeVideoList) + \
                        '\nMedian=' + GetMedian(self.decodeVideoList) + \
                        '\nAverage=' + GetAverage(self.decodeVideoList)
            axs[0][1].plot(dVideoFrames, self.decodeVideoList, label=statistics2)
            axs[0][1].legend(loc='upper right', fontsize=6, shadow=True, fancybox=True)
            axs[0][1].set_xlabel('video frame index')
            axs[0][1].set_ylabel('decode time (ms)')
            axs[0][1].grid(True)

        if len(self.pxpConvVideoList):
            pxpVideoFrames = np.arange(0, len(self.pxpConvVideoList), dt)
            statistics3 = 'Max=' + GetMax(self.pxpConvVideoList) + \
                        '\nMin=' + GetMin(self.pxpConvVideoList) + \
                        '\nMedian=' + GetMedian(self.pxpConvVideoList) + \
                        '\nAverage=' + GetAverage(self.pxpConvVideoList)
            axs[1][1].plot(pxpVideoFrames, self.pxpConvVideoList, label=statistics3)
            axs[1][1].legend(loc='upper right', fontsize=6, shadow=True, fancybox=True)
            axs[1][1].set_xlabel('video frame index')
            axs[1][1].set_ylabel('PXP Convert time (ms)')
            axs[1][1].grid(True)

        if len(self.lcdDispVideoList):
            lcdVideoFrames = np.arange(0, len(self.lcdDispVideoList), dt)
            statistics4 = 'Max=' + GetMax(self.lcdDispVideoList) + \
                        '\nMin=' + GetMin(self.lcdDispVideoList) + \
                        '\nMedian=' + GetMedian(self.lcdDispVideoList) + \
                        '\nAverage=' + GetAverage(self.lcdDispVideoList)
            axs[2][1].plot(lcdVideoFrames, self.lcdDispVideoList, label=statistics4)
            axs[2][1].legend(loc='upper right', fontsize=6, shadow=True, fancybox=True)
            axs[2][1].set_xlabel('video frame index')
            axs[2][1].set_ylabel('LCD Display time (ms)')
            axs[2][1].grid(True)

        fig.tight_layout()
        plt.show()

    def run(self):
        # Read command line arguments.
        args = self._read_options()
        if not len(args.input):
            print ("Error: FFmpeg decode time data file required")
            return 1
        if not len(args.vinfo):
            print ("Error: Video resolution info required")
            return 1
        if not len(args.build):
            print ("Error: Project build info required")
            return 1
        self._inputFile = args.input
        self._videoInfo = args.vinfo
        self._projBuild = args.build
        self._process()
        self._show()
        self._finalize()

# Create the main class and run it.
if __name__ == "__main__":
    exit(ShowFFmpegTime().run())

import sys, os
import argparse
import numpy as np
import matplotlib.pyplot as plt

def GetMax(lst):
   return max(lst)

def GetMin(lst):
   return min(lst)

def GetMedian(lst):
   data = sorted(lst)
   size = len(lst)
   if size % 2 == 0:
      median = (data[size//2]+data[size//2-1])/2
   if size % 2 == 1:
      median = data[(size-1)//2]
   return median

def GetAverage(lst):
   sum = 0.0
   for item in lst:
      sum += item
   return sum/len(lst)

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
        self.playVideoList = []

        with open(self._inputFile, 'r') as fileObj:
            try:
                while(True):
                    # one line frame data like this:
                    #        read            decode audio       decode video        play video
                    # '0x0000000000006000,0x0000000000000000,0x0000100000006000,0x0000200000006000,'
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

                        playVideoIndex = frameData.find(',', decodeVideoIndex + 1)
                        if playVideoIndex != -1:
                            playVideoTime = int(frameData[decodeVideoIndex + 3:playVideoIndex], 16)
                            if playVideoTime:
                                self.playVideoList.extend([playVideoTime / 1000000.0])
                    else:
                        break
            finally:
                fileObj.close()

    def _finalize(self):
        print ("Processing is completed!")

    def _show(self):
        dt = 1
        fig, axs = plt.subplots(4, 1)
        axs[0].set_title(self._projBuild, color='b')
        #axs[0].set_title('Text & Data & Heap & Stack & frameBuffer in SDRAM', color='b')
        #axs[0].set_title('Text in FlexRAM,  Data & Heap & Stack & frameBuffer in SDRAM', color='b')
        #axs[0].set_title('Text in Hyper, Data & Heap & Stack & frameBuffer in SDRAM', color='b')
        #axs[0].set_title('Text in Hyper, Data in FlexRAM, Heap & Stack & frameBuffer in SDRAM', color='b')

        readFrames = np.arange(0, len(self.readFrameList), dt)
        axs[0].plot(readFrames, self.readFrameList)
        vinfo = 'media frame(' + self._videoInfo  + ') index \n'
        statistics0 = 'Max=' + str(GetMax(self.readFrameList)) + \
                    ', Min=' + str(GetMin(self.readFrameList)) + \
                    ', Median=' + str(GetMedian(self.readFrameList)) + \
                    ', Average=' + str(GetAverage(self.readFrameList))
        axs[0].set_xlabel(vinfo + statistics0)
        axs[0].set_ylabel('read time (us)')
        axs[0].grid(True)

        audioFrames = np.arange(0, len(self.decodeAudioList), dt)
        axs[1].plot(audioFrames, self.decodeAudioList)
        statistics1 = 'Max=' + str(GetMax(self.decodeAudioList)) + \
                    ', Min=' + str(GetMin(self.decodeAudioList)) + \
                    ', Median=' + str(GetMedian(self.decodeAudioList)) + \
                    ', Average=' + str(GetAverage(self.decodeAudioList))
        axs[1].set_xlabel('audio frame index \n' + statistics1)
        axs[1].set_ylabel('decode time (ms)')
        axs[1].grid(True)

        dVideoFrames = np.arange(0, len(self.decodeVideoList), dt)
        axs[2].plot(dVideoFrames, self.decodeVideoList)
        statistics2 = 'Max=' + str(GetMax(self.decodeVideoList)) + \
                    ', Min=' + str(GetMin(self.decodeVideoList)) + \
                    ', Median=' + str(GetMedian(self.decodeVideoList)) + \
                    ', Average=' + str(GetAverage(self.decodeVideoList))
        axs[2].set_xlabel('video frame index \n' + statistics2)
        axs[2].set_ylabel('decode time (ms)')
        axs[2].grid(True)

        pVideoFrames = np.arange(0, len(self.playVideoList), dt)
        axs[3].plot(pVideoFrames, self.playVideoList)
        statistics3 = 'Max=' + str(GetMax(self.playVideoList)) + \
                    ', Min=' + str(GetMin(self.playVideoList)) + \
                    ', Median=' + str(GetMedian(self.playVideoList)) + \
                    ', Average=' + str(GetAverage(self.playVideoList))
        axs[3].set_xlabel('video frame index \n ' + statistics3)
        axs[3].set_ylabel('play time (ms)')
        axs[3].grid(True)

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

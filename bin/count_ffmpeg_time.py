import sys, os
import argparse
import numpy as np
import matplotlib.pyplot as plt

class ShowFFmpegTime(object):
    def __init__(self):
        pass

    def _read_options(self):
        parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter)
        parser.add_argument("input", help="FFmpeg decode time data file."),
        return parser.parse_args()

    def _process(self):
        self.readFrameList = []
        self.decodeAudioList = []
        self.decodeVideoList = []

        with open(self._inputFile, 'r') as fileObj:
            try:
                while(True):
                    # one line frame data like this:
                    # '0x0000000000006000,0x0000000000000000,0x0000100000006000,'
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
                    else:
                        break
            finally:
                fileObj.close()

    def _finalize(self):
        print ("Processing is completed!")

    def _show(self):
        dt = 1
        fig, axs = plt.subplots(3, 1)
        axs[0].set_title('Text & Data & Heap & Stack & frameBuffer in SDRAM', color='b')
        #axs[0].set_title('Text in FlexRAM,  Data & Heap & Stack & frameBuffer in SDRAM', color='b')
        #axs[0].set_title('Text in Hyper, Data & Heap & Stack & frameBuffer in SDRAM', color='b')
        #axs[0].set_title('Text in Hyper, Data in FlexRAM, Heap & Stack & frameBuffer in SDRAM', color='b')

        readFrames = np.arange(0, len(self.readFrameList), dt)
        axs[0].plot(readFrames, self.readFrameList)
        axs[0].set_xlabel('media frame(720HD) index')
        axs[0].set_ylabel('read time (us)')
        axs[0].grid(True)

        audioFrames = np.arange(0, len(self.decodeAudioList), dt)
        axs[1].plot(audioFrames, self.decodeAudioList)
        axs[1].set_xlabel('audio frame index')
        axs[1].set_ylabel('decode time (ms)')
        axs[1].grid(True)

        videoFrames = np.arange(0, len(self.decodeVideoList), dt)
        axs[2].plot(videoFrames, self.decodeVideoList)
        axs[2].set_xlabel('video frame index')
        axs[2].set_ylabel('decode time (ms)')
        axs[2].grid(True)

        fig.tight_layout()
        plt.show()

    def run(self):
        # Read command line arguments.
        args = self._read_options()
        if not len(args.input):
            print ("Error: FFmpeg decode time data file required")
            return 1
        self._inputFile = args.input
        self._process()
        self._show()
        self._finalize()

# Create the main class and run it.
if __name__ == "__main__":
    exit(ShowFFmpegTime().run())

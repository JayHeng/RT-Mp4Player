import sys, os
import argparse
import Image

class ConvJpegToRgb888(object):
    def __init__(self):
        pass

    def _read_options(self):
        parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter)
        parser.add_argument("-v", "--version", help="jpeg2rgb888 0.1")
        parser.add_argument("-o", "--output", required=True, metavar="PATH", type=argparse.FileType('wb'), help="Specify the output file.")
        parser.add_argument("input", help="JPEG Image file/folder."),
        return parser.parse_args()

    def _process(self):
        imgFiles = []

        if os.path.isfile(self._inputFile):
            imgFiles.append(self._inputFile)
        elif os.path.isdir(self._inputFile):
            imgFolder = os.path.abspath(self._inputFile)
            inputFiles = os.listdir(imgFolder)
            for idx in range(len(inputFiles)):
                imgFiles.append(os.path.join(imgFolder, inputFiles[idx]))
        else:
            return

        for fileIdx in range(len(imgFiles)):
            imgObj = Image.open(imgFiles[fileIdx])
            pixelBuf = imgObj.getdata()
            print "\r\nConverting image file ", fileIdx
            print "Image size: ", imgObj.size
            print "Image Pixel Mode: ", imgObj.mode
            for i in range(len(pixelBuf)):
                for j in range(len(pixelBuf[i])):
                    self._outputFile.write(chr(pixelBuf[i][len(pixelBuf[i]) - j - 1]))
        self._outputFile.close()

    def _finalize(self):
        print ("Processing is completed!")

    def run(self):
        # Read command line arguments.
        args = self._read_options()

        if not len(args.input):
            print ("Error: JPEG Image file/folder required")
            return 1

        if args.output is None:
            print("Error: failed to open output file")
            return 1

        self._inputFile = args.input
        self._outputFile = args.output

        self._process()
        self._finalize()

# Create the main class and run it.
if __name__ == "__main__":
    exit(ConvJpegToRgb888().run())

import sys, os
import argparse

s_loopDivider = list(range(27, 55))
s_postDivider = [1, 2, 4, 8, 16]
s_lcdifPreDiv = list(range(0, 8))
s_lcdifDiv = list(range(0, 8))

s_maxPixelClockFreq = 75000000

kLcdResolution_480x272 = '480x272'
kLcdResolution_800x600 = '800x600'
kLcdResolution_1280x600 = '1280x800'

s_lcdMagic_480x272  = (480 + 41 + 4 + 18) * (272 + 10 + 4 + 2)
s_lcdMagic_800x600  = (800 + 48 + 88 + 112) * (600 + 3 + 39 + 21)
s_lcdMagic_1280x800 = (1280 + 10 + 80 + 70) * (800 + 3 + 10 + 10)

class FindBestDividers(object):
    def __init__(self):
        pass

    def _read_options(self):
        parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter)
        parser.add_argument("-v", "--version", help="FindBestDividers 0.1")
        parser.add_argument("frequency", help="Desired LCD frequency."),
        parser.add_argument("resolution", help="Target LCD resolution."),
        return parser.parse_args()

    def _process(self):
        #               OSC24M * loopDivider / postDivider
        #  freq = --------------------------------------------
        #            lcdifDiv * lcdifPreDiv * s_lcdMagic_xxx

        desiredLcdFreq =  int(self._freq)
        actualLcdFreq = 0
        actualPixelFreq = 0

        freqError = int(self._freq)

        loopDivider = 0
        postDivider = 0
        lcdifPreDiv = 0
        lcdifDiv = 0

        for i in s_loopDivider:
            for j in s_postDivider:
                for k in s_lcdifPreDiv:
                    for l in s_lcdifDiv:
                        pixelClockFreq = 24000000.0 * i / j / (k + 1) / (l + 1)
                        if pixelClockFreq < s_maxPixelClockFreq:
                            freq = pixelClockFreq / self._lcdMagic
                            error = 0
                            if freq > desiredLcdFreq:
                                error = freq - desiredLcdFreq
                            else:
                                error =  desiredLcdFreq - freq
                            if error < freqError:
                                freqError = error
                                actualLcdFreq = freq
                                actualPixelFreq = pixelClockFreq
                                loopDivider = i
                                postDivider = j
                                lcdifPreDiv = k
                                lcdifDiv = l
                            else:
                                pass

        print "---------------------------------------------"
        print "loopDivider: ", loopDivider
        print "postDivider: ", postDivider
        print "lcdifPreDiv: ", lcdifPreDiv
        print "   lcdifDiv: ", lcdifDiv

        print "\r\nDesired Lcd Refresh Freq (Hz): ", desiredLcdFreq
        print " Actual Lcd Refresh Freq (Hz): ", actualLcdFreq
        print "Actual Pixel Clock Freq (MHz): ", actualPixelFreq / 1000000
        print "---------------------------------------------"

    def _finalize(self):
        pass

    def run(self):
        # Read command line arguments.
        args = self._read_options()

        if args.frequency is None:
            print ("Error: You have to specify LCD frequency")
            return 1
        else:
            self._freq = args.frequency

        if args.resolution is None:
            print ("Error: You have to specify LCD resolution")
            return 1
        else:
            if args.resolution == kLcdResolution_480x272:
                self._lcdMagic = s_lcdMagic_480x272
            elif args.resolution == kLcdResolution_800x600:
                self._lcdMagic = s_lcdMagic_800x600
            elif args.resolution == kLcdResolution_1280x600:
                self._lcdMagic = s_lcdMagic_1280x800
            else:
                print ("Error: You have to set proper LCD resolution (eg, 480x272, 800x600, 1280x800)")
                return 1

        self._process()
        self._finalize()

# Create the main class and run it.
if __name__ == "__main__":
    exit(FindBestDividers().run())

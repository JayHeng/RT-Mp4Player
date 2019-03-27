import sys
import os
import argparse
import binascii
import string

# usage: gen_dcd_code_array.py dcd.bin -o dcd.c


##
# @brief Command-line interface to the tool.
class ConvertBinToCcodeArray(object):
    def __init__(self):
        pass

    def _read_options(self):
        parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter)
        parser.add_argument("-v", "--version", help="bin2ccodearray 0.1.0")
        parser.add_argument("-o", "--output", required=True, metavar="PATH", type=argparse.FileType('w+'), help="Specify the output file.")
        parser.add_argument("input", help="image bin file."),

        return parser.parse_args()

    def _getByteListFromInputFile(self):
        byte_list = []

        with open(self._inputFile, 'rb') as srcFile:
            try:
                while(True):
                    byteRead = srcFile.read(1)
                    if not byteRead:
                        break;
                    #bytestr = str(hex(ord(byteRead)))
                    hexstr = str(binascii.hexlify(byteRead))
                    bytestr = '0x' + hexstr[2:4]
                    byte_list.append(bytestr)

            finally:
                srcFile.close()
            return byte_list


    def _process(self):
        byte_list = self._getByteListFromInputFile()
        if len(byte_list) < 1:
            print ("Invalid bin file!")

        self._outputFile.writelines("const uint8_t dcd_data[] = {")

        row_count = len(byte_list)
        for i in range(0, row_count):
            if (i % 0x10) == 0:
                self._outputFile.writelines("\n")

            if (i == row_count - 1):
                self._outputFile.writelines(byte_list[i])
            else:
                self._outputFile.writelines(byte_list[i] + ',')

        self._outputFile.writelines("\n};")
        self._outputFile.close()


    def _finalize(self):
        print ("Convertion is completed!")


    def run(self):
        # Read command line arguments.
        args = self._read_options()

        if not len(args.input):
            print ("Error: bin file required")
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
    exit(ConvertBinToCcodeArray().run())

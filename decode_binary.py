import argparse
import os
import sys
import re
import struct

class ArdusatBinaryData(object):
    ARDUSAT_SENSOR_TYPE_ACCELERATION = b'\x00'
    ARDUSAT_SENSOR_TYPE_MAGNETIC = b'\x01'
    ARDUSAT_SENSOR_TYPE_GYRO = b'\x02'
    ARDUSAT_SENSOR_TYPE_ORIENTATION = b'\x03'
    ARDUSAT_SENSOR_TYPE_TEMPERATURE = b'\x04'
    ARDUSAT_SENSOR_TYPE_LUMINOSITY = b'\x05'
    ARDUSAT_SENSOR_TYPE_UV = b'\x06'
    ARDUSAT_SENSOR_TYPE_PRESSURE = b'\x07'

    STRUCTURE_SIZE = { ARDUSAT_SENSOR_TYPE_ACCELERATION: (18, "<BIfff"),
                       ARDUSAT_SENSOR_TYPE_MAGNETIC: (18, "<BIfff"),
                       ARDUSAT_SENSOR_TYPE_GYRO: (18, "<BIfff"),
                       ARDUSAT_SENSOR_TYPE_TEMPERATURE: (10, "<BIf"),
                       ARDUSAT_SENSOR_TYPE_LUMINOSITY: (10, "<BIf"),
                       ARDUSAT_SENSOR_TYPE_UV: (10, "<BIf"),
                       ARDUSAT_SENSOR_TYPE_ORIENTATION: (18, "<BIfff"),
                       ARDUSAT_SENSOR_TYPE_PRESSURE: (10, "<BIf"),
    }

    SENSOR_NAME = { ARDUSAT_SENSOR_TYPE_ACCELERATION: "accelerometer",
                    ARDUSAT_SENSOR_TYPE_MAGNETIC: "magnetic",
                    ARDUSAT_SENSOR_TYPE_GYRO: "gyro",
                    ARDUSAT_SENSOR_TYPE_TEMPERATURE: "temperature",
                    ARDUSAT_SENSOR_TYPE_LUMINOSITY: "luminosity",
                    ARDUSAT_SENSOR_TYPE_UV: "uv",
                    ARDUSAT_SENSOR_TYPE_ORIENTATION: "orientation",
                    ARDUSAT_SENSOR_TYPE_PRESSURE: "pressure",
    }

    def __init__(self, input_file, halt_on_error=False):
        self.input_file = input_file
        self.lines = 0
        self.halt_on_error = halt_on_error

    def __iter__(self):
        return self

    def next(self):
        return self.__next__()

    def __next__(self):
        """
        Looks forward in the input file to process another "line" of input 
        (binary data structure). 

        :return: 0 if successful, 1 if end of file, -1 if error
        """
        first_byte = self.input_file.read(1)
        if first_byte == b"":
            raise StopIteration

        try:
            strut_size = self.STRUCTURE_SIZE[first_byte]
        except KeyError:
            # Check if we have a timestamp header
            if first_byte == b'\xFF' and input_file.read(1) == b'\xFF':
                ts1, ts2 = struct.unpack("<II", input_file.read(8))
                return "timestamp: %d at millis %d\n" % (ts1, ts2)
            pos = self.input_file.tell()
            err = "Unknown sensor type %#x found at byte %d!" % \
                  (ord(first_byte), pos)
            if self.halt_on_error:
                raise LookupError(err)
            else:
                print(err)
                return ""

        # Read in binary data. We've already read the first byte (sensor type),
        # so read struct_size - 1 bytes
        data = struct.unpack(strut_size[1], \
                             self.input_file.read(strut_size[0] - 1))
        output_string = "%d,%s,%d" % \
                        (data[1], self.SENSOR_NAME[first_byte], data[0])
        for val in data[2:]:
            output_string += ",%f" % val
        self.lines += 1
        return "%s\n" % output_string

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Decodes a binary data file " \
                                     "created using the ArdusatSDK.")
    parser.add_argument("-o,--output-file", nargs=1, dest="output_file", \
                        help="CSV file to write decoded data to")
    parser.add_argument("-s,--stop-on-error", action="store_true",
                        dest="halt_on_error",
                        help="Stop decoding if unexpected bytes encountered")
    parser.add_argument("input_file", help="Binary data file to decode")
    args = parser.parse_args()

    # check if input file exists
    if not os.path.isfile(args.input_file):
        print("You must provide a binary data file to decode!!!")
        sys.exit(1)

    # if we don't have an output_file given, make one out of the input file path
    if not args.output_file:
        match = re.search("([^\.]*)\.[a-z]*", os.path.basename(args.input_file))
        if match is None:
            print("Unable to create output file path. " \
                  "Try specifying with the -o option")
            sys.exit(1)
        args.output_file = os.path.join(os.path.dirname(args.input_file),
                                        "%s.%s" % (match.group(1), "csv"))
    else:
        args.output_file = args.output_file[0]

    print("Decoding file %s (%d bytes) and saving data to %s..." %
          (args.input_file, os.path.getsize(args.input_file), args.output_file))

    with open(args.input_file, "rb") as input_file:
        with open(args.output_file, "w") as output_file:
            data = ArdusatBinaryData(input_file, args.halt_on_error)
            for line in data:
                output_file.write(line)

    print("Finished decoding %s, saved %d data observations to %s" %
          (args.input_file, data.lines, args.output_file))

    sys.exit(0)

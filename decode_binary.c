/**
 * @file   decode_binary.c
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 19, 2014
 * @brief  Utility to decode binary data saved using the Ardusat SDK.
 *
 *         Takes a path to a binary data file as input and outputs a CSV file
 *         with the data.
 */
#ifndef ARDUINO

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <getopt.h>

#include "utility/BinaryDataFmt.h"

static struct option cli_options[] = {
  { "output-file", required_argument, NULL, 'o' },
  { "help", no_argument, NULL, 'h' },
  { 0, 0, 0, 0}
};

void print_usage(char *argv [])
{
  printf("Decodes a binary data file created using the ArdusatSDK.\n");
  printf("usage: %s [options] FILE\n", argv[0]);
  printf("Options:\n");
  printf("  -o,--output-file PATH          CSV file to write decoded data to\n");
  printf("  -h,--help                      Print this usage info.\n");
}

#define err_print_usage(err) err; print_usage(argv); return -1

int check_input_file_path(const char *input_path)
{
  int len = strlen(input_path);
  int i;
  char ext[4];

  memcpy(ext, &input_path[len - 4], 4);
  for (i = 0; i < 4; ++i) {
    ext[i] = tolower(ext[i]);
  }

  return strncmp(ext, ".bin", 4);
}

char * make_output_csv_path_from_input(const char *input_path)
{
  int len = strlen(input_path);
  const char * ptr = input_path + len;
  char * output_file_path;
  int i;

  while (ptr != input_path && *ptr != '/') {
    ptr--;
  }

  len = strlen(ptr) + 1;
  output_file_path = (char *) malloc(len + 3);

  strncpy(output_file_path, ptr, len);

  // Regardless of the path we found, make sure it starts with "./" to
  // save in the current working directory
  if (*output_file_path != '/') {
    for (i = len; i >= 0; --i) {
      output_file_path[i + 1] = output_file_path[i];
    }
    output_file_path[0] = '/';
    len++;
  }

  for (i = len; i >= 0; --i) {
    output_file_path[i + 1] = output_file_path[i];
  }
  output_file_path[0] = '.';
  len++;

  output_file_path[len - 4] = 'c';
  output_file_path[len - 3] = 's';
  output_file_path[len - 2] = 'v';

  output_file_path[len - 1] = '\0';

  return output_file_path;
}

#define get_data_struct(typeSize, name) \
  if (fread(buf + 1, 1, typeSize - 1, input) == 0) { \
    return -1; \
  } \
  timestamp = *((uint32_t *)(buf + 2)); \
  sensor_id = *((uint8_t *)(buf + 1)); \
  sprintf(sensor_name, name); \

int process_next_row(FILE *input, FILE *output)
{
  char buf[100];
  char sensor_name[100];
  char val_buf[100];
  uint8_t sensor_id;
  uint32_t timestamp;
  uint32_t timestamp_2;

  if (fread(buf, 1, 1, input) == 0) {
    return -1;
  }

  // Unfortunately, we can't just rely on the struct definitions in BinaryDataFmt.h
  // to unpack the binary data due to structure alignment issues, especially on
  // 64 bit machines.
  switch(buf[0]) {
    case(ARDUSAT_SENSOR_TYPE_ACCELERATION):
      get_data_struct(18, "acceleration")
      sprintf(val_buf, "%f,%f,%f", *(float *)(buf + 6),
              *(float *)(buf + 10), *(float *)(buf + 14));
      break;
    case(ARDUSAT_SENSOR_TYPE_MAGNETIC):
      get_data_struct(18, "magnetic")
      sprintf(val_buf, "%f,%f,%f", *(float *)(buf + 6),
              *(float *)(buf + 10), *(float *)(buf + 14));
      break;
    case(ARDUSAT_SENSOR_TYPE_GYRO):
      get_data_struct(18, "gyro")
      sprintf(val_buf, "%f,%f,%f", *(float *)(buf + 6),
              *(float *)(buf + 10), *(float *)(buf + 14));
      break;
    case(ARDUSAT_SENSOR_TYPE_TEMPERATURE):
      get_data_struct(10, "temperature")
      sprintf(val_buf, "%f", *(float *)(buf + 6));
      break;
    case(ARDUSAT_SENSOR_TYPE_LUMINOSITY):
      get_data_struct(10, "luminosity")
      sprintf(val_buf, "%f", *(float *)(buf + 6));
      break;
    case(ARDUSAT_SENSOR_TYPE_UV):
      get_data_struct(10, "uv")
      sprintf(val_buf, "%f", *(float *)(buf + 6));
      break;
    case (ARDUSAT_SENSOR_TYPE_ORIENTATION):
      get_data_struct(18, "orientation")
      sprintf(val_buf, "%f,%f,%f", *(float *)(buf + 6),
              *(float *)(buf + 10), *(float *)(buf + 14));
      break;
    case (ARDUSAT_SENSOR_TYPE_PRESSURE):
      get_data_struct(10, "pressure")
      sprintf(val_buf, "%f", *(float *)(buf + 6));
      break;
    case ((char) 0xFF):
      // check if timestamp header
      if (fread(buf + 1, 1, 1, input) == 0 || buf[1] != (char) 0xFF ||
          fread(buf + 2, 1, 8, input) == 0 ) {
        return -1;
      }
      timestamp = *((uint32_t *)(buf + 2));
      timestamp_2 = *((uint32_t *)(buf + 6));
      return fprintf(output, "timestamp: %u at millis %u\n", timestamp, timestamp_2) <= 0;
    default:
      printf("Unknown sensor type %d found!\n", buf[0]);
      return -1;
  }

  return fprintf(output, "%d,%s,%d,%s\n", timestamp, sensor_name, sensor_id, val_buf) <= 0;
}

int main(int argc, char *argv[])
{
  int c;
  int option_idx;
  char *output_file_path = NULL;
  char *input_file_path = NULL;
  FILE *input_file;
  FILE *output_file;
  int lines = 0;
  int ret;

  if (argc < 2) {
    err_print_usage(printf("You need to provide a binary data file to decode!!!\n"));
  }

  while ((c = getopt_long(argc, argv, "ho:", cli_options, &option_idx)) != -1) {
    switch(c) {
      case 'h':
        print_usage(argv);
        return 0;
      case 'o':
        output_file_path = optarg;
        break;
    }
  }

  if (optind == argc) {
    err_print_usage(printf("You need to provide a binary data file to decode!!!\n"));
  }

  input_file_path = argv[optind++];

  if (check_input_file_path(input_file_path) != 0) {
    err_print_usage(printf("Invalid input file path given.\n"));
  }

  if (output_file_path == NULL)
    output_file_path = make_output_csv_path_from_input(input_file_path);

  printf("Decoding file %s and saving data to %s...\n", input_file_path, output_file_path);

  input_file = fopen(input_file_path, "rb");
  if (!input_file) {
    err_print_usage(printf("Error opening input file %s\n", input_file_path));
  }

  output_file = fopen(output_file_path, "w");
  if (!output_file) {
    err_print_usage(printf("Could not open file %s for writing.\n", output_file_path));
  }

  while (!feof(input_file)) {
    if (process_next_row(input_file, output_file) != 0) {
      break;
    }
    lines++;
  }


  if (feof(input_file)) {
    printf("Finished decoding %s. Saved %d data observations to %s.\n",
           input_file_path, lines, output_file_path);
    ret = 0;
  } else {
    long pos = ftell(input_file);
    printf("Uh oh, something went wrong reading %s at %ld\n", input_file_path, pos);
    ret = -1;
  }

  fclose(input_file);
  fclose(output_file);

  return ret;
}
#endif

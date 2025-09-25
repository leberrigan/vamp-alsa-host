#ifndef AIRSPYMinder_HPP
#define AIRSPYMinder_HPP

#include <string>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <memory>
#include <cmath>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

using namespace std;

#include "DevMinder.hpp"

extern "C" {
  // struct that airspy_tcp sends down the data stream
  // each such struct is followed by (size - sizeof(airspy_stream_segment_hdr_t) bytes of I/Q sample data.
typedef struct {
        uint32_t size;   // size of this header plus number of sample bytes before next header
        double ts;       // timestamp of first sample in stream
} airspy_stream_segment_hdr_t;
};

class AIRSPYMinder : public DevMinder {

protected:

  int                    numFD;       // number of file descriptors required for polling on this device
  int                    airspytcp;      // fd for connection to airspy_tcp server via unix domain socket; -1 means not connected
  struct sockaddr_un     airspytcpAddr;  // address for airspy_tcp server
  std::string            socketPath;  // filesystem path to airspy_tcp unix domain socket
  airspy_stream_segment_hdr_t   header;      // most recently encountered header in stream
  bool                   headerValid; // is content of latestHeader valid?
  unsigned int           segi;        // how many bytes from this segment (header + data) have been processed, including those from the header
  unsigned int           bytesAvail;  // bytes available in recv buffer, from latest ioctl()

public:

  const static int AIRSPY_FRAMES = 2048; // number of complex samples to process per buffer cycle. 
                                          // rtl-sdr is set to 2048 for 240ksps, airspy samples at 6msps, 
                                          // so use a larger buffer size to more closely match the time covered per cycle. 
                                          // It's not an exact match because the default segment size for airspy is 65536 bytes (16384 complex samples).
  const static int MAX_SEGMENT_SIZE = 8192; // Max segment size is AIRSPY_FRAMES * 4, rounded up to allow for buffer overflow
  const static int SAMPLE_SCALE = 1;   // Airspy already provides 16-bit packets so no scaling needed.
                                       // amount by which to multiply signed 8-bit samples to get signed 16-bit sample; for plugins, this
                                       // only matters if downsampling by averaging (and then, only improves precision a bit);
                                       // simple subsampling isn't affected, as the scale
                                       // (max absolute value) of the samples incorporates this factor.  Raw outputs, such as audio
                                       // listening on the web interface, and recording of .wav files, are affected.

  virtual int hw_open();

  virtual bool hw_is_open();

  AIRSPYMinder(const string &devName, int rate, unsigned int numChan, const string &label, double now);

  ~AIRSPYMinder();

  virtual int hw_getNumPollFDs ();

  virtual int hw_getPollFDs (struct pollfd *pollfds);

  virtual int hw_handleEvents ( struct pollfd *pollfds, bool timedOut);

  virtual int hw_getFrames (int16_t *buf, int numFrames, double & frameTimestamp);

protected:

  virtual void delete_privates();
  virtual int hw_do_start();
  virtual int hw_do_stop();
  virtual int hw_do_restart();
  virtual bool hw_running(double timeNow);

  int getHWRateForRate(int rate); // get minimum sampling rate that is an integer multiple of desired rate; this is the hardware
  // sampling rate that nodejs would have set for this airspy device
  // sets fields hwRate and downsamplefactor correspondingly; returns 0 on sucess, non-zero on error.

};

#endif // AIRSPYMinder_HPP

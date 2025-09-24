#include "AIRSPYMinder.hpp"
#include <sys/ioctl.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <time.h>

#define UNIX_PATH_MAX 108

void AIRSPYMinder::delete_privates() {
  if (airspytcp >= 0) {
    close(airspytcp);
    airspytcp = -1;
  }
};

int AIRSPYMinder::hw_open() {
  airspytcp = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (airspytcp < 0) {
    std::cerr << "unable to open socket fd for airspy\n";
    return 1;
  }
  memset(& airspytcpAddr, 0, sizeof(struct sockaddr_un));
  airspytcpAddr.sun_family = AF_UNIX;
  snprintf(airspytcpAddr.sun_path, UNIX_PATH_MAX, socketPath.c_str());
  if(connect(airspytcp, (struct sockaddr *) &airspytcpAddr, sizeof(struct sockaddr_un))) {
    std::cerr << "unable to connect to socket for airspy\n";
    return 2;
  }
  return getHWRateForRate(rate);
};

bool AIRSPYMinder::hw_is_open() {
  return airspytcp >= 0;
};

int AIRSPYMinder::hw_do_stop() {
  return 0;
};

bool AIRSPYMinder::hw_running(double timeNow) {
  return airspytcp > 0 && timeNow - lastDataReceived < 2.0;
};

int AIRSPYMinder::hw_do_start() {
  if (airspytcp < 0 && open())
    return 1;
  hasError = 0;
  return 0;
}

int AIRSPYMinder::hw_do_restart() {
  hasError = 0;
  return 0;
};

AIRSPYMinder::AIRSPYMinder(const string &devName, int rate, unsigned int numChan, const string &label, double now):
  DevMinder(devName, rate, numChan, 128 * SAMPLE_SCALE, label, now, AIRSPY_FRAMES),
  numFD(1),
  airspytcp(-1),
  headerValid(false),
  segi(0),
  bytesAvail(0)
{
  if (devName.substr(0, 7) != "airspy:")
    throw std::runtime_error("Invalid name for AIRSPY device; must look like 'airspy:PATH'");
  socketPath = devName.substr(7);
};

AIRSPYMinder::~AIRSPYMinder() {
  delete_privates();
};

int AIRSPYMinder::hw_getNumPollFDs () {
  return airspytcp >= 0 ? numFD : 0;
};

int AIRSPYMinder::hw_getPollFDs (struct pollfd *pollfds) {
  if (airspytcp < 0)
    return 1;

  pollfds->fd = airspytcp;
  pollfds->events = POLLIN | POLLPRI;
  return 0;
}

int AIRSPYMinder::hw_handleEvents ( struct pollfd *pollfds, bool timedOut) {
  if (airspytcp < 0 || timedOut)
    return 0;
  if (pollfds->revents & POLLIN) {
    // return number of frames available
    int avail;
    ioctl(airspytcp, FIONREAD, &avail);
    bytesAvail = avail;
    return (avail + 1) / 2; // hardcoded: 1 byte per sample, two channels (I/Q)
  }
  return 0;
};

int AIRSPYMinder::hw_getFrames (int16_t *buf, int numFrames, double & frameTimestamp) {
  /*
    data available in the recv buf look like so:

     [ seg header, or tail thereof ]? [sample data] [ seg header ] [sample data] ... [seg header] [sample data]

    and there is no alignment to stream segment boundaries as we're using a stream-oriented socket protocol

    So we perform one or more recv()'s to completely use up the available byte count (recorded in the
    most recent call to hw_handleEvents), removing any stream_segment_hdr_t structures; the latest such
    struct is saved for use of its timestamp and byte count.  We return the count of frames copied to
    buf, which will not exceed numFrames.

  */
  int sampleBytesCopied = 0;

  // we accumulate the best estimate of the timestamp for the first sample in this segment,
  // getting a separate estimate for each segment header in the current buffer.

  int numTSest = 0;
  if (segi >= sizeof(stream_segment_hdr_t)) {
    // we already have a header for the current segment, so
    // estimate the timestamp of the first sample to be copied to the buffer
    frameTimestamp = header.ts + ((segi - sizeof(stream_segment_hdr_t)) / 2.0) / hwRate;
    numTSest = 1;
  } else {
    frameTimestamp = 0;
  }

  while (bytesAvail > 0) {
    // try finish filling in the current stream_segment_hdr_t, if not already full.

    int hdrBytes  = std::min((int) sizeof(stream_segment_hdr_t) - (int) segi, (int) bytesAvail);
    if (hdrBytes > 0) {
      // need to try finish filling in header
      int bytes = recv(airspytcp, ((char *) (& header)) + segi, hdrBytes, 0);
      if (bytes != hdrBytes)
        std::cerr << "Bytes = " << bytes << " but hdrBytes = " << hdrBytes << std::endl;
      bytesAvail -= bytes;
      segi +=  bytes;
      // if new header has been obtained, add a new estimate of the timestamp for the first sample in the buffer
      if (segi == sizeof(stream_segment_hdr_t)) {
        frameTimestamp += header.ts - (sampleBytesCopied / 2.0) / hwRate;
        ++numTSest;
      }
      continue;
    }

    // try finish the sample data from the stream segment

    int dataBytes = std::min((int) header.size - (int) segi, (int) bytesAvail);
    if (dataBytes > 0) {
      // need to continue copying data
      int bytes = recv(airspytcp, buf, dataBytes, 0);
      if (bytes != dataBytes)
        std::cerr << "Bytes = " << bytes << " but dataBytes = " << dataBytes << std::endl;

      // expand the samples from 8 to 16 bits
      // working from right to left; also, shift left 8 bits so sample downsampling using average method maintains more precision.
      int16_t * ebuf = buf + (bytes - 1);
      unsigned char * cbuf = ((unsigned char *) buf) + (bytes - 1);
      for ( int i = bytes; i > 0; --i, --cbuf, --ebuf)
        *ebuf = ((int16_t) (int8_t) (*cbuf - 128)) * SAMPLE_SCALE; // scale from 8 to 16 bits, improving precision of averaging downsampling

      bytesAvail -= bytes;
      segi +=  bytes;
      sampleBytesCopied += bytes;
      buf += bytes;
      if (segi == header.size) {
        segi = 0;
      }
      continue;
    }
  }
  if (numTSest > 1)
    frameTimestamp /= numTSest;
  return sampleBytesCopied / 2; // returning # of frames
};

int
AIRSPYMinder::getHWRateForRate(int rate) {
  // airspy allows sampling rates in these ranges:
  // [225 001 - 300 000] and [900 001 - 3 200 000]

  if (rate <= 0 || rate > 3200000)
    return 1;

  hwRate = rate;
  for(;;) {
    if ((hwRate >= 225001 && hwRate <= 300000) || (hwRate >= 900001 && hwRate <= 3200000))
      break;
    hwRate += rate;
  }
  return 0;
};

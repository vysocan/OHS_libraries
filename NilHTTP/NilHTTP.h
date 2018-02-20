/**
 *       
 */
#include <NilGPRS.h>

/* Result codes */
enum Result {
  SUCCESS = 0,
  ERROR_INITIALIZATION = 1,
  ERROR_BEARER_PROFILE_GPRS = 2,
  ERROR_BEARER_PROFILE_APN = 3,
  ERROR_OPEN_GPRS_CONTEXT = 4,
  ERROR_QUERY_GPRS_CONTEXT = 5,
  ERROR_CLOSE_GPRS_CONTEXT = 6,
  ERROR_HTTP_INIT = 7,
  ERROR_HTTP_CID = 8,
  ERROR_HTTP_PARA = 9,
  ERROR_HTTP_GET = 10,
  ERROR_HTTP_READ = 11,
  ERROR_HTTP_CLOSE = 12,
  ERROR_HTTP_POST = 13,
  ERROR_HTTP_DATA = 14,
  ERROR_HTTP_CONTENT = 15,
  ERROR_NORMAL_MODE = 16,
  ERROR_LOW_CONSUMPTION_MODE = 17,
  ERROR_HTTPS_ENABLE = 18,
  ERROR_HTTPS_DISABLE = 19
};


class HTTP : public NilGPRS {

  public:
    HTTP(unsigned long);
    Result configureBearer(const char *apn);
    Result connect();
    Result disconnect();
    Result get(const char *uri, char *response);
    Result post(const char *uri, const char *body, char *response);
    void sleep();
    void wakeUp();

  private:
    void readResponse(char *response);
    Result setHTTPSession(const char *uri);
    void parseJSONResponse(const char *buffer, unsigned int bufferSize, char *response);
};

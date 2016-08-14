#ifndef SCANNER_H_
#define SCANNER_H_

#include <memory>

class scanner {
     public:
      scanner(std::string dev, int baud);
      ~scanner();

      int sendQuery(unsigned char dataMode, unsigned char pid, std::string& rawResponse);
      int sendQuery(unsigned char dataMode, unsigned char pid);
      int fullQuery(unsigned char mode);
      int dtcErrors();
      void displayDTC();
      float getVoltage();
      int availablePid(); // TODO

     private:
         class impl;
         std::unique_ptr<impl> m_impl;
};
#endif /* SCANNER_H_ */



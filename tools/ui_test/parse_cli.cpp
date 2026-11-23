// Reads one message per line, prints the firmware parser's verdict.
#include <stdio.h>
#include <string.h>
#include "ControlProtocol.h"
int main() {
  char line[256];
  setvbuf(stdout, nullptr, _IOLBF, 0);
  while (fgets(line, sizeof line, stdin)) {
    size_t n = strlen(line);
    if (n && line[n-1] == '\n') line[--n] = 0;
    Message m = parseMessage(line, n);
    const char* t[] = {"Invalid","Drive","Arm","SpeedLimit","Stop","Claim","Leds","Feel","Ping"};
    printf("%s %g %g\n", t[(int)m.type], m.a, m.b);
  }
}

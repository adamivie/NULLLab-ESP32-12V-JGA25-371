#include "net_log.h"

NetLog Log;

void NetLog::begin() {
  server_.begin();
  server_.setNoDelay(true);
  started_ = true;
}

size_t NetLog::write(const uint8_t* data, size_t len) {
  Serial.write(data, len);
  portENTER_CRITICAL(&lock_);
  for (size_t i = 0; i < len; ++i) buffer_[(written_ + i) % kBufferSize] = data[i];
  written_ += len;
  portEXIT_CRITICAL(&lock_);
  return len;
}

void NetLog::loop() {
  if (!started_) return;

  if (server_.hasClient()) {
    // Newest connection wins, so a stale session never locks you out.
    if (client_) client_.stop();
    client_ = server_.accept();
    client_.setNoDelay(true);
    portENTER_CRITICAL(&lock_);
    sent_ = written_ > kBufferSize ? written_ - kBufferSize : 0;  // replay the backlog
    portEXIT_CRITICAL(&lock_);
  }
  if (!client_) return;
  if (!client_.connected()) {
    client_.stop();
    return;
  }
  while (client_.available() > 0) client_.read();  // input is ignored

  // Copy a chunk out under the lock, send it outside the lock.
  uint8_t chunk[256];
  size_t n = 0;
  portENTER_CRITICAL(&lock_);
  if (written_ - sent_ > kBufferSize) sent_ = written_ - kBufferSize;  // fell behind: skip ahead
  while (n < sizeof(chunk) && sent_ != written_) chunk[n++] = buffer_[sent_++ % kBufferSize];
  portEXIT_CRITICAL(&lock_);
  if (n > 0) client_.write(chunk, n);
}

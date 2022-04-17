double capRange(double min, double val, double max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

double normaliseValueToRange(double min, double val, double max) {
  double range = max - min;
  while (val > max) val -= range;
  while (val < min) val += range;
  return val;
}

class Buffer {
  int maxLength;
  byte* buffer;
  int pointer;

  public:
    Buffer(int maxLength) {
      this->maxLength = maxLength;
      this->buffer = new byte(maxLength);
      this->pointer = 0;
    }

    void write(byte b) {
      if (this->pointer == this->maxLength) {
        this->send();
      }
      this->buffer[this->pointer] = b;
      this->pointer += 1;
    }

    void send() {
      Serial.write(this->buffer, pointer);
      Serial.send_now();
      this->pointer = 0;
    }
};

#pragma once
#ifndef SERIAL_HH
#define SERIAL_HH

#include <string>
#include <vector>
#include <cstdint>

class Serial {
 private:
  std::string dev_path;   // armazenar por valor para evitar dangling reference
  void *priv = nullptr;

  // torna estas funções privadas, as assinaturas permanecem
  void *configure_serial();
  const std::vector<uint8_t> readBytes() const;
  void writeBytes(const std::vector<uint8_t> &vec) const;

 public:
  Serial();
  explicit Serial(const std::string &dev_path);
  ~Serial();

  // operadores para conveniência (síncronos, retornam dados lidos/escritos)
  inline Serial &operator>>(std::string &s) {
    std::vector<uint8_t> v = this->readBytes();
    std::string(v.cbegin(), v.cend()).swap(s);
    return *this;
  }

  inline Serial &operator<<(const std::string &s) {
    this->writeBytes(std::vector<uint8_t>(s.cbegin(), s.cend()));
    return *this;
  }

  // checker simples
  inline bool ok() const { return this->priv != nullptr; }
};

#endif // SERIAL_HH

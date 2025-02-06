#ifndef CPU_INFO_COMPUTERUNIT_H
#define CPU_INFO_COMPUTERUNIT_H

#include <cstdint>

class ISystemInformation {
public:
  ISystemInformation() = default;
  virtual ~ISystemInformation() = default;
  virtual int64_t GetTotalMemory() = 0;
  virtual int64_t GetTotalUsageMemory() = 0;
  virtual double GetCpuTotalUsage() = 0;
};

class SystemInformation : public ISystemInformation {

public:
  SystemInformation();
  virtual ~SystemInformation() override;

  int64_t GetTotalMemory() override;
  int64_t GetTotalUsageMemory() override;
  double GetCpuTotalUsage() override;

private:
  struct PImpl;

  PImpl *m_impl;
};

class ComputerUnit {

private:
  long max_memory;
  long mem_usage;
  double cpu_sys_usage;
  double mem_usage_percentage;

public:
  ComputerUnit() {
    SystemInformation sys_info;

    max_memory = sys_info.GetTotalMemory();
    mem_usage = sys_info.GetTotalUsageMemory();
    cpu_sys_usage = sys_info.GetCpuTotalUsage();
  }
  void print_info();
};

#endif // CPU_INFO_COMPUTERUNIT_H

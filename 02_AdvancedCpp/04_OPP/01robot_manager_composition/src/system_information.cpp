#include "robot_manager_composition/system_information.h"
#include <iostream>
#include "sys/sysinfo.h"
#include "sys/times.h"
#include "sys/types.h"
#include "sys/vtimes.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

void ComputerUnit::print_info() {
  printf("Total CPU usage: %.2f%%\n", cpu_sys_usage);
  try {
    auto mem_usage_percentage = (float)mem_usage / (float)max_memory * 100.0;
    printf("Total memory usage: %.2f%%\n", mem_usage_percentage);
  } catch (std::logic_error e) {
    std::cerr << e.what();
  }
}


struct SystemInformation::PImpl : public ISystemInformation {
public:
  PImpl() = default;

  ~PImpl() override = default;

  int64_t GetTotalMemory() override {
    struct sysinfo memInfo;

    sysinfo(&memInfo);
    long long totalPhysMem = memInfo.totalram;
    // Multiply in next statement to avoid int overflow on right hand side...
    totalPhysMem *= memInfo.mem_unit;

    return totalPhysMem;
  }

  int64_t GetTotalUsageMemory() override {
    struct sysinfo memInfo;

    sysinfo(&memInfo);
    long long physMemUsed = memInfo.totalram - memInfo.freeram;
    // Multiply in next statement to avoid int overflow on right hand side...
    physMemUsed *= memInfo.mem_unit;

    return physMemUsed;
  }

  double GetCpuTotalUsage() override {
    double percent;
    FILE *file;
    unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

    file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow,
           &totalSys, &totalIdle);
    fclose(file);

    if (totalUser < m_lastTotalUser || totalUserLow < m_lastTotalUserLow ||
        totalSys < m_lastTotalSys || totalIdle < m_lastTotalIdle) {
      // Overflow detection. Just skip this value.
      percent = -1.0;
    } else {
      total = (totalUser - m_lastTotalUser) +
              (totalUserLow - m_lastTotalUserLow) + (totalSys - m_lastTotalSys);
      percent = total;
      total += (totalIdle - m_lastTotalIdle);
      percent /= total;
      percent *= 100;
    }

    m_lastTotalUser = totalUser;
    m_lastTotalUserLow = totalUserLow;
    m_lastTotalSys = totalSys;
    m_lastTotalIdle = totalIdle;

    return percent;
  }

private:
  void init() {
    FILE *file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &m_lastTotalUser,
           &m_lastTotalUserLow, &m_lastTotalSys, &m_lastTotalIdle);
    fclose(file);
  }

private:
  unsigned long long m_lastTotalUser;
  unsigned long long m_lastTotalUserLow;
  unsigned long long m_lastTotalSys;
  unsigned long long m_lastTotalIdle;
};


SystemInformation::SystemInformation() { m_impl = new PImpl(); }

SystemInformation::~SystemInformation() noexcept { delete m_impl; }

int64_t SystemInformation::GetTotalMemory() { return m_impl->GetTotalMemory(); }

int64_t SystemInformation::GetTotalUsageMemory() {
  return m_impl->GetTotalUsageMemory();
}

double SystemInformation::GetCpuTotalUsage() {
  return m_impl->GetCpuTotalUsage();
}

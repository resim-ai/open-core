

#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <iostream>

void run_binary(char *cmd, char *args[]) {
  int pid = fork();
  if (pid == 0) {
    int res = execv(cmd, args);

    if (res == -1) {
      std::cerr << "Failed to start " << cmd << std::endl;
      std::cerr << "Error: " << errno << std::endl;
      exit(1);
    }
    exit(0);
  }
  int status;
  waitpid(pid, &status, 0);
}

int main(int argc, char *argv[]) {
  {
    char *cmd = "/sim_container_entrypoint.runfiles/resim/simulator/resim_run";
    char *args[] = {
        "resim_run",
        "-c",
        "/tmp/resim/inputs/experience.sim",
        "-l",
        "/tmp/resim/outputs/resim_log.mcap",
        NULL};
    run_binary(cmd, args);
  }
  {
    char *cmd =
        "/sim_container_entrypoint.runfiles/resim/visualization/log/"
        "make_visualization_log";
    char *args[] = {
        "make_visualization_log",
        "-l",
        "/tmp/resim/outputs/resim_log.mcap",
        "-o",
        "/tmp/resim/outputs/vis.mcap",
        NULL};
    run_binary(cmd, args);
  }
  return EXIT_SUCCESS;
}

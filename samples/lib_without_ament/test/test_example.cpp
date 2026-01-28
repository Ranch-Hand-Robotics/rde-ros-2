#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <unistd.h>

#include "rde_lib_without_ament/example.hpp"

// Test fixture
class ExampleLibraryTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Capture stdout by redirecting file descriptor 1 to a pipe
    fflush(stdout);
    pipe(pipe_fd);
    old_stdout = dup(STDOUT_FILENO);
    dup2(pipe_fd[1], STDOUT_FILENO);
    close(pipe_fd[1]);
  }

  void TearDown() override {
    // Restore stdout
    fflush(stdout);
    dup2(old_stdout, STDOUT_FILENO);
    close(old_stdout);
    close(pipe_fd[0]);
  }

  std::string GetCapturedOutput() {
    // Read from pipe
    std::string output;
    char buffer[256];
    ssize_t bytes_read;
    while ((bytes_read = read(pipe_fd[0], buffer, sizeof(buffer))) > 0) {
      output.append(buffer, bytes_read);
    }
    return output;
  }

  int pipe_fd[2];
  int old_stdout;
};

// Test that print_example function exists and can be called
TEST_F(ExampleLibraryTest, PrintExampleFunctionExists) {
  EXPECT_NO_THROW(print_example());
}

// Test that print_example produces expected output
TEST_F(ExampleLibraryTest, PrintExampleOutput) {
  print_example();
  std::string output = GetCapturedOutput();
  EXPECT_NE(output.find("This function comes from a non-ament c++ library"), std::string::npos);
}

// Test that the library is callable multiple times
TEST_F(ExampleLibraryTest, MultipleCallsWork) {
  EXPECT_NO_THROW({
    print_example();
    print_example();
    print_example();
  });
}

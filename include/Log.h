#ifndef LOG_H
#define LOG_H

#include <string>
#include <iostream>
#include <vector>

namespace Logger {

  /**
   * A utility class for logging
   * It's capable to provide coloured output and a timestamp with every message
   */
  class Log {
  public:
    /**
     * The unicode codes for some colours
     * FG_* states for foreground colours
     * BG_* states for background colours
     */
    enum Colour {
      FG_DEFAULT          = 39,
      FG_BLACK            = 30,
      FG_RED              = 31,
      FG_GREEN            = 32,
      FG_YELLOW           = 33,
      FG_BLUE             = 34,
      FG_MAGENTA          = 35,
      FG_CYAN             = 36,
      FG_LIGHT_GRAY       = 37,
      FG_DARK_GRAY        = 90,
      FG_LIGHT_RED        = 91,
      FG_LIGHT_GREEN      = 92,
      FG_LIGHT_YELLOW     = 93,
      FG_LIGHT_BLUE       = 94,
      FG_LIGHT_MAGENTA    = 95,
      FG_LIGHT_CYAN       = 96,
      FG_WHITE            = 97,

      BG_DEFAULT          = 49,
      BG_RED              = 41,
      BG_GREEN            = 42,
      BG_BLUE             = 44,
    };

  public:
    /**
     * Logs a message using simple output, i.e. no colours
     * @param msg The message to output
     */
    static void plain(const std::string& msg);

    /**
     * Logs an info message (blue)
     * @param msg The message to output
     */
    static void info(const std::string& msg);

    /**
     * Logs an error message (red)
     * @param msg The message to output
     */
    static void error(const std::string& msg);

    /**
     * Logs a success message (green)
     * @param msg The message to output
     */
    static void success(const std::string& msg);

    /**
     * Logs a video message (yellow)
     * @param msg The message to output
     */
    static void video(const std::string& msg);

    /**
     * Logs a templated std::vector
     * @param vec The vector to log
     */
    template<typename T> static void vector(const std::vector<T>& vec);

    /**
     * Logs a templated plain 16 items array as a matrix
     * @param matrix The matrix to log
     */
    template<typename T> static void matrix(const T* matrix);

    /**
     * Retrieves the current date and time
     * @return a string holding the current date and time
     */
    static const std::string currentDateTime();

    /**
     * Sets whether the log should be coloured or not
     * False by default
     * @param coloured Whether the log should be coloured or not
     */
    static void setColouredOutput(bool coloured);

  private:
    static bool coloured_output; ///< Whether the log should be coloured or not
  };

  template<typename T>
  void Log::vector(const std::vector<T>& vec) {
    std::cout << currentDateTime() << " [ ";
    typename std::vector<T>::const_iterator i;
    for(i = vec.begin(); i != vec.end(); ++i)
      std::cout << *i << ' ';
    std::cout << "] " << std::endl;
  }

  template<typename T>
  void Log::matrix(const T* matrix) {
    for(int i = 0; i < 16; i += 4) {
      if(i < 4) {
        std::cout << currentDateTime() << " [ ";
      }
      else {
        std::cout << "                      [ ";
      }
      std::cout << matrix[i] << " " << matrix[i+1] << " " << matrix[i+2] << " " << matrix[i+3];
      std::cout << " ] " << std::endl;
    }
  }

}

#endif

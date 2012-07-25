#ifndef KEYBOARD_CONSOLE_LISTENER_HPP_
#define KEYBOARD_CONSOLE_LISTENER_HPP_

#include <stdlib.h>
#include <iostream>
#include <termios.h>
#include <signal.h>
#include <cstring>

#include <boost/function.hpp>

namespace cob_3d_mapping_tools
{
  class KeyboardConsoleListener
  {
  public:
    struct KEYS
    {
      enum KeyCode
      {
        A = 0x61,
        D = 0x64,
        S = 0x73,
        W = 0x77,
        Q = 0x71,
        E = 0x65,

        A_CAP = 0x41,
        D_CAP = 0x44,
        S_CAP = 0x53,
        W_CAP = 0x57,
        Q_CAP = 0x51,
        E_CAP = 0x45
      };
    };

  private:
    KeyboardConsoleListener();
    // prevent compiler generating methods:
    KeyboardConsoleListener(KeyboardConsoleListener const& copy); // Not implemented
    KeyboardConsoleListener& operator=(KeyboardConsoleListener const& copy); // Not implemented

  public:
    ~KeyboardConsoleListener() {reset();}

    static KeyboardConsoleListener& get()
      {
        // The only instance
        // Guaranteed to be lazy initialized
        // Guaranteed that it will be destroyed correctly
        static KeyboardConsoleListener instance;
        return instance;
      }
    static void reset(int sig=0);

    void init();
    char spinOnce();
    void spin();
    void waitForIt(char c);
    bool hasStopped();
    void registerKeyboardEventCallback(boost::function<void (char)>& f);

  private:

    struct termios old_conf, new_conf;

    int kfd;
    bool tc_modified_;
    bool has_stopped_;
    char c;

    boost::function<void (char)> callback;

  };



  /******************************************/
  /*************** Definition ***************/
  /******************************************/

  KeyboardConsoleListener::KeyboardConsoleListener()
    : kfd(0), tc_modified_(false), has_stopped_(true)
  { }

  void KeyboardConsoleListener::init()
  {
    // get the console in raw mode
    tcgetattr(kfd, &old_conf);
    new_conf = old_conf;
    //memcpy(&raw, &cooked, sizeof(struct termios));
    new_conf.c_lflag &= (~ICANON & ~ECHO);
    // Setting a new line, then end of file
    new_conf.c_cc[VEOL] = 1;
    new_conf.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &new_conf);
    tc_modified_ = true;
  }

  char KeyboardConsoleListener::spinOnce()
  {
    if(read(kfd, &c, 1) < 0)
    {
      std::cerr << "ERR: read():" << std::endl;
      KeyboardConsoleListener::reset();
    }
    if(callback) callback(c);
    return c;
  }

  void KeyboardConsoleListener::spin()
  {
    has_stopped_=false;
    while(!has_stopped_)
    {
      spinOnce();
    }
    KeyboardConsoleListener::reset();
  }

  void KeyboardConsoleListener::waitForIt(char c)
  {
    for(;;) { if (spinOnce() == c) return; };
  }

  bool KeyboardConsoleListener::hasStopped()
  {
    return has_stopped_;
  }

  void KeyboardConsoleListener::registerKeyboardEventCallback(boost::function<void (char)>& f)
  {
    callback = f;
  }

  void KeyboardConsoleListener::reset(int sig)
  {
    if (get().tc_modified_)
    {
      tcsetattr(get().kfd, TCSANOW, &(get().old_conf));
      get().has_stopped_=true;
      get().tc_modified_ = false;
    }
  }
}

/*
  int main(int argc, char** argv)
  {
  KeyboardConsoleListener::get().init();
  KeyboardConsoleListener::get().spin();
  return 0;
  }
*/
#endif //KEYBOARD_CONSOLE_LISTENER_HPP_

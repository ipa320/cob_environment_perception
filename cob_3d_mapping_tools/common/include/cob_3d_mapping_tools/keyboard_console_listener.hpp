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
        ENTER = 0x0D,
        SPACE = 0x20,

        A_CAP = 0x41,
        B_CAP, C_CAP, D_CAP, E_CAP, F_CAP, G_CAP, H_CAP, I_CAP, J_CAP, K_CAP, L_CAP, M_CAP, N_CAP,
        O_CAP, P_CAP, Q_CAP, R_CAP, S_CAP, T_CAP, U_CAP, V_CAP, W_CAP, X_CAP, Y_CAP, Z_CAP, //=0x5A

        A = 0x61,
        B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z, //=0x7A
      };
    };

  private:
    KeyboardConsoleListener();
    // prevent compiler generating methods:
    KeyboardConsoleListener(KeyboardConsoleListener const& copy); // Not implemented
    KeyboardConsoleListener& operator=(KeyboardConsoleListener const& copy); // Not implemented

  public:
    ~KeyboardConsoleListener() { reset(); }

    static KeyboardConsoleListener& get()
      {
        // The only instance
        // Guaranteed to be lazy initialized
        // Guaranteed that it will be destroyed correctly
        static KeyboardConsoleListener instance;
        return instance;
      }
    static void reset(int sig=0);

    void init(unsigned int timeout=1); // timeout * 0.1 sec [1..255]
    char spinOnce();
    void spin();
    void waitForIt(char c);
    char waitForIt(char* c, int size);
    bool hasStopped();
    void registerKeyboardEventCallback(boost::function<void (char)>& f);

  private:

    struct termios old_conf, new_conf;

    int kfd;
    bool tc_modified_;
    bool has_stopped_;

    boost::function<void (char)> callback;

  };



  /******************************************/
  /*************** Definition ***************/
  /******************************************/

  KeyboardConsoleListener::KeyboardConsoleListener()
    : kfd(0), tc_modified_(false), has_stopped_(true)
  { }

  void KeyboardConsoleListener::init(unsigned int timeout)
  {
    // get the console in raw mode
    tcgetattr(kfd, &old_conf);

    new_conf = old_conf;
    //memcpy(&raw, &cooked, sizeof(struct termios));
    const bool debug_b = false;
    if (debug_b)
    {
      std::cout <<"---Old Config---"<< std::endl;
      std::cout <<"ECHO:   "<< ( (new_conf.c_lflag & ECHO) == ECHO ? "True" : "False") << std::endl;
      std::cout <<"ECHOE:  "<< ( (new_conf.c_lflag & ECHOE) == ECHOE ? "True" : "False") << std::endl;
      std::cout <<"ECHOK:  "<< ( (new_conf.c_lflag & ECHOK) == ECHOK ? "True" : "False") << std::endl;
      std::cout <<"ECHONL: "<< ( (new_conf.c_lflag & ECHONL) == ECHONL ? "True" : "False") << std::endl;
      std::cout <<"ICANON: "<< ( (new_conf.c_lflag & ICANON) == ICANON ? "True" : "False") << std::endl;
      std::cout <<"IEXTEN: "<< ( (new_conf.c_lflag & IEXTEN) == IEXTEN ? "True" : "False") << std::endl;
      std::cout <<"ISIG:   "<< ( (new_conf.c_lflag & ISIG) == ISIG ? "True" : "False") << std::endl;
      std::cout <<"NOFLSH: "<< ( (new_conf.c_lflag & NOFLSH) == NOFLSH ? "True" : "False") << std::endl;
      std::cout <<"TOSTOP: "<< ( (new_conf.c_lflag & TOSTOP) == TOSTOP ? "True" : "False") << std::endl;
    }
    new_conf.c_lflag &= (~ICANON & ~ECHO);
    if (debug_b)
    {
      std::cout <<"---New Config---"<< std::endl;
      std::cout <<"ECHO:   "<< ( (new_conf.c_lflag & ECHO) == ECHO ? "True" : "False") << std::endl;
      std::cout <<"ECHOE:  "<< ( (new_conf.c_lflag & ECHOE) == ECHOE ? "True" : "False") << std::endl;
      std::cout <<"ECHOK:  "<< ( (new_conf.c_lflag & ECHOK) == ECHOK ? "True" : "False") << std::endl;
      std::cout <<"ECHONL: "<< ( (new_conf.c_lflag & ECHONL) == ECHONL ? "True" : "False") << std::endl;
      std::cout <<"ICANON: "<< ( (new_conf.c_lflag & ICANON) == ICANON ? "True" : "False") << std::endl;
      std::cout <<"IEXTEN: "<< ( (new_conf.c_lflag & IEXTEN) == IEXTEN ? "True" : "False") << std::endl;
      std::cout <<"ISIG:   "<< ( (new_conf.c_lflag & ISIG) == ISIG ? "True" : "False") << std::endl;
      std::cout <<"NOFLSH: "<< ( (new_conf.c_lflag & NOFLSH) == NOFLSH ? "True" : "False") << std::endl;
      std::cout <<"TOSTOP: "<< ( (new_conf.c_lflag & TOSTOP) == TOSTOP ? "True" : "False") << std::endl;
    }
    // Setting a new line, then end of file
    //new_conf.c_lflag = 0;
    //new_conf.c_cc[VEOL] = 1;
    //new_conf.c_cc[VEOF] = 2;
    new_conf.c_cc[VTIME] = std::max(std::min(timeout, (unsigned int)255), (unsigned int)1);
    new_conf.c_cc[VMIN] = 0;
    tcsetattr(kfd, TCSANOW, &new_conf);
    tc_modified_ = true;
  }

  char KeyboardConsoleListener::spinOnce()
  {
    //std::cout << "BF spinOnce"<<std::endl;
    char c;
    int res=read(kfd, &c, 1);
    if(res < 0)
    {
      std::cerr << "ERR: read():" << std::endl;
      KeyboardConsoleListener::reset();
    }
    else if(res == 0) return NULL;
    if (callback) callback(c);
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

  char KeyboardConsoleListener::waitForIt(char* c, int size)
  {
    //std::cout<<"BF waitForIt"<<std::endl;
    for(;;)
    {
      char c_in = spinOnce();
      //std::cout << (int)c_in << std::endl;
      for(int i=0; i<size; ++i) { if(c_in == c[i]) { return c_in; } }
    }
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

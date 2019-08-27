#ifndef __MY_LOG_H__
#define __MY_LOG_H__
#include<stdio.h>
#include<string>
#include <stdarg.h>
#include<string.h>
namespace WE_LOG
{
  using std::string;

  enum WELog_LEVEL{
    WELog_Null,
    WELog_Error,
    WELog_Warning,
    WELog_Debug,
    WELog_Info,
    WELog_Everythig
  };


  class WELog
  {
  public:
    WELog()
    {
      fileName = "a.log";
      log_level = WELog_Info;
      file = NULL;
      enableLog = true;
      isOpened = false;
      setLogFile("canCmd.log");

    }
    ~WELog()
    {
      if(file)
      {
        fclose(file);
        isOpened = false;
      }
    }

    bool setLogFile(const char* name)
    {
      if(strcmp(fileName, name) == 0)
        return true;
      if(isOpened)
      {
        fclose(file);
        file =NULL;
        fileName = NULL;
        isOpened = false;
      }


      file = fopen(name,"w");
      if(file)
      {
        isOpened = true;
        fileName = name;
        return true;
      }
      return false;
    }

    const char* getLogFile()
    {
      return fileName;
    }
    void setWELogLevel(WELog_LEVEL lev)
    {
      log_level = lev;
    }

    WELog_LEVEL getWELogLevel()
    {
      return log_level;
    }


    int logDirect(WELog_LEVEL lev, bool putScreen, const char* fmt, ...)
    {
      if(lev > log_level || !enableLog || !isOpened)
        return 0;

      va_list ap;
      va_start( ap, fmt );

      char format[201]= {0};
      int len = strlen(fmt);
      const char* p;
      if(len < 200)
      {
        strcpy(format, fmt);
        if(format[len - 1] != '\n')
          format[len] = '\n';
        p = (const char* )format;
      }
      else
      {
        p = fmt;
      }

      if(putScreen)
        vfprintf(stdout, p, ap);

      int e = vfprintf(file,p,ap);
      va_end(ap);
      fflush(file);
      return e;
    }

    int logError(const char* fmt, ...)
    {
        if( log_level < WELog_Error)
            return 0;

        va_list ap;
        va_start( ap, fmt );

        char format[201]= {0};
        int len = strlen(fmt);
        const char* p;
        if(len < 200)
        {
          strcpy(format, fmt);
          if(format[len - 1] != '\n')
            format[len] = '\n';
          p = (const char* )format;
        }
        else
        {
          p = fmt;
        }

        //if(putScreen)
          vfprintf(stdout, p, ap);

        int e = vfprintf(file,p,ap);
        va_end(ap);
        fflush(file);
        return e;

    }
    int logWarning( bool putScreen, const char* fmt, ...)
    {
        if(log_level < WELog_Warning)
            return 0;

        va_list ap;
        va_start( ap, fmt );

        char format[201]= {0};
        int len = strlen(fmt);
        const char* p;
        if(len < 200)
        {
          strcpy(format, fmt);
          if(format[len - 1] != '\n')
            format[len] = '\n';
          p = (const char* )format;
        }
        else
        {
          p = fmt;
        }

        if(putScreen)
          vfprintf(stdout, p, ap);

        int e = vfprintf(file,p,ap);
        va_end(ap);
        fflush(file);
        return e;
    }
    int logDebug( bool putScreen, const char* fmt, ...)
    {
        if(log_level < WELog_Debug)
            return 0;

        va_list ap;
        va_start( ap, fmt );

        char format[201]= {0};
        int len = strlen(fmt);
        const char* p;
        if(len < 200)
        {
          strcpy(format, fmt);
          if(format[len - 1] != '\n')
            format[len] = '\n';
          p = (const char* )format;
        }
        else
        {
          p = fmt;
        }

        if(putScreen)
          vfprintf(stdout, p, ap);

        int e = vfprintf(file,p,ap);
        va_end(ap);
        fflush(file);
        return e;
    }
    int logInfo( bool putScreen, const char* fmt, ...)
    {
        if(log_level < WELog_Info)
            return 0;


        va_list ap;
        va_start( ap, fmt );

        char format[201]= {0};
        int len = strlen(fmt);
        const char* p;
        if(len < 200)
        {
          strcpy(format, fmt);
          if(format[len - 1] != '\n')
            format[len] = '\n';
          p = (const char* )format;
        }
        else
        {
          p = fmt;
        }

        if(putScreen)
          vfprintf(stdout, p, ap);

        int e = vfprintf(file,p,ap);
        va_end(ap);
        fflush(file);
        return e;
    }
    int logEverythin( bool putScreen, const char* fmt, ...)
    {
        if(log_level < WELog_Everythig)
            return 0;

        va_list ap;
        va_start( ap, fmt );

        char format[201]= {0};
        int len = strlen(fmt);
        const char* p;
        if(len < 200)
        {
          strcpy(format, fmt);
          if(format[len - 1] != '\n')
            format[len] = '\n';
          p = (const char* )format;
        }
        else
        {
          p = fmt;
        }

        if(putScreen)
          vfprintf(stdout, p, ap);

        int e = vfprintf(file,p,ap);
        va_end(ap);
        fflush(file);
        return e;
    }

  private:
    bool enableLog;
    bool isOpened;
    const char* fileName;
    WELog_LEVEL log_level;
    FILE* file;
  };
}

#endif

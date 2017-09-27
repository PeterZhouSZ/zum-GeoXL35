#ifndef PrefixOutput_H
#define PrefixOutput_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PrefixOutput
{
public:
    PrefixOutput(void) {}

    void push(const std::string& str)
    {
        str_prefix.push_back(str);
    }
    void pop(void) { str_prefix.pop_back(); }
    void clear(void) { str_prefix.clear(); }

    void write(const std::string& str = "")
    {
        writePrefix();
        debugOutput << str;
    }
    void PrefixOutput::operator<<(const std::string& str)
    {
        debugOutput << str;
    }

private:
    void writePrefix(void)
    {
        for (const std::string& sp : str_prefix) {
            debugOutput << sp;
        }
    }

public:
    std::deque<std::string> str_prefix;
};

extern PrefixOutput prefixOutput;

#endif

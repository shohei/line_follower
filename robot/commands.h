#ifndef COMMANDS_H
#define COMMANDS_H

enum class CommandName
{
    Forward,
    Backward,
    TurnLeft,
    TurnRight,
};

class Command
{
public:
    CommandName name;
    int value;
    static void dump(const Command commands[], int length);
    static void run(const Command commands[], int length);
};

#endif
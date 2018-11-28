#ifndef BEHAVIOR_H
#define BEHAVIOR_H


class Behavior {
  public:
    Behavior(const char* n, const int l);
    Behavior(int l);

  // Getters and Setters
    int getId();

  private:
    int id;
    const char* name;

};

#endif //BEHAVIOR_H

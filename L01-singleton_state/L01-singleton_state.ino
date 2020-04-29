class State {
  public:
  virtual ~State(){}
  virtual void enter(){};
  virtual State* run() = 0;
};

class White: public State {
  public:
  virtual void enter() override;
  virtual State* run() override;
};

class Green: public State {
  public:
  virtual void enter() override;
  virtual State* run() override;
};

static White white;
static Green green;
unsigned long lastMillis = 0;

void flash(int pin) {
   for(int i = 0; i<20; ++i) {
    digitalWrite(pin,!digitalRead(pin));
    delay(30);
  } 
}

State* White::run() {
  if (millis() < lastMillis + 1000) {
    digitalWrite(13,HIGH);
    return this;      
  }
  else {
    lastMillis = millis();
    digitalWrite(13,LOW);
    return &green;  
  }
}

void White::enter() {
  flash(13);
  // update the timer
  lastMillis = millis();
}

State* Green::run() {
  if (millis() < lastMillis + 2000) {
    digitalWrite(12,HIGH);
    return this;      
  }
  else {
    lastMillis = millis();
    digitalWrite(12,LOW);
    return &white;  
  }
}

void Green::enter() {
  flash(12);
  // update the timer
  lastMillis = millis();
}

void setup() {
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
}

bool isChanged = false;
State* state = &white;
State* lastState = state;
void loop() {
  // if state changed last time, call enter
  if (isChanged)
    lastState->enter();
  // call run repeatedly
  state = lastState->run();
  // if state changed, record it;
  isChanged = (state != lastState);
  // reset lastState to current
  lastState = state;
}

#include <Servo.h>
#include "notes.h"
#include "debug.h"

// CONCEPT:
// Timeline represents sequence of events;
// Actor represents object capable of do something useful at some point in time - execute event;
// Stage represents logical time segment;
//   stage has two phases - preparation and execution
//   during preparation actors produce initial sequence of events on timeline;
//   during execution as time passes Stage takes events from timeline in accordance to their timestamps;
//   every time event is executed actor may produce more events on the timeline;
//   once timeline contains no events Stage is considered completed;
// Scenario represents a number of consequent Stages happening one by one;
//   every stage is defined by a preparation function which declares actors producing initial sequence of events;
//   once last stage is completed scenario is considered ended;

// VALUES BELOW ARE HARDWARE LIMITS
// DO NOT CHANGE WITHOUT MANUAL TESTING
// ####################################
namespace Limits
{
const int ServoArmUpper = 700;
const int ServoArmLower = 2000;

const int ServoCapUpper = 1150;
const int ServoCapLower = 1900;
}

namespace Pins
{
const byte Vibro = 6;
const byte Piezo = 7;
const byte Toggle = 8;
const byte Arm = 9;
const byte Cap = 10;
}
// ####################################



namespace Angles
{
const int ArmTargetDefault = Limits::ServoArmUpper;
const int ArmTargetSwitchedOff = 900;
const int ArmTargetSwitchedOn = 980;
const int ArmTargetPokeAround = 1050;
const int ArmTargetSlow = ArmTargetPokeAround;

const int ArmUnderCap = 1600;
const int ArmRestDefault = 1800;
const int ArmRestSlow = Limits::ServoArmLower;
const int ArmRestSwing = 1400;
const int ArmRestSwingLast = 1700;



const int CapTargetDefault = 1420;
const int CapTargetSlow = Limits::ServoCapUpper;
const int CapTargetSwing = 1350;

const int CapRestDefault = 1550;
const int CapRestSlow = Limits::ServoCapLower;
}

namespace Speeds
{
const int ArmRapid = 250;
const int ArmSlow = 2500;
const int CapSlow = 5000;

const int ArmDefault = 500;
const int CapDefault = 800;
const int CapFlap = 400;
}

namespace Timeouts
{
const unsigned long SERVO_POSITION_WAIT_USEC = 160000;
const unsigned long TOGGLE_SWITCH_WAIT_USEC = 10000;
const unsigned long PAUSE_1_SEC_USEC = 1000000;
const unsigned long PAUSE_3_SEC_USEC = 3000000;
const unsigned long PAUSE_1O_SEC_USEC = 10000000;
const unsigned long PAUSE_1OO_MSEC_USEC = 100000;
const unsigned long PAUSE_2OO_MSEC_USEC = 200000;
const unsigned long PAUSE_3OO_MSEC_USEC = 300000;
const unsigned long TOGGLE_DEBOUNCE_DURATION_USEC = 10000;
}

namespace Details
{

using Timestamp = unsigned long;

template< class T >
void swap(T& l, T& r)
{
  int tmp = l;
  l = r;
  r = tmp;
}

template< class TPayload >
struct HeapImpl
{
  struct Node
  {
    Node(const Timestamp val, TPayload* payload)
      : _val(val)
      , _payload(payload)
      , _next(0)
      , _prev(0)
    {}
    const Timestamp _val;
    TPayload* _payload;
    Node* _next;
    Node* _prev;
  };

  Node _head;
  Node _tail;
  Node* _free;

  HeapImpl()
    : _head(-1, 0)
    , _tail(-1, 0)
    , _free(0)
  {
    _head._next = &_tail;
    _tail._prev = &_head;
  }
  bool empty() const
  {
    return _head._next == &_tail;
  }
  Timestamp topVal() const
  {
    return _head._next->_val;
  }
  TPayload* topPayload() const
  {
    return _head._next->_payload;
  }
  void drop()
  {
    while(not empty())
    {
      pop();
    }
  }
  void pop()
  {
    if(not empty())
    {
      Node* node = _head._next;
      Node* top = _head._next->_next;
      _head._next = top;
      top->_prev = &_head;
      delete node;
    }
  }
  void insert(const Timestamp val, TPayload* payload)
  {
    //DEBUG( "insert usec=%ld ptr=%ld", val, (unsigned long) payload );

    Node* node = new Node(val, payload);
    Node* prev = _tail._prev;
    while(prev != &_head and prev->_val > node->_val)
    {
      prev = prev->_prev;
    }
    Node* next = prev->_next;
    node->_prev = prev;
    node->_next = next;
    prev->_next = node;
    next->_prev = node;
  }
};

struct ActorBase
{
  ActorBase()
    : _usec(0)
  {}
  virtual ~ActorBase()
  {}
  void setDelay(const Timestamp usec)
  {
    _usec = usec;
  }
  void do_loop(const Timestamp now, HeapImpl< ActorBase >& timeline)
  {
    if(not do_loop(now))
    {
      timeline.insert(now + _usec, this);
    }
  }
  virtual bool do_loop(const Timestamp now) = 0;

  Timestamp _usec;
};

using Heap = HeapImpl< ActorBase >;
}

using Timeline = Details::Heap;
using Timestamp = Details::Timestamp;

namespace Components
{

// http://gammon.com.au/switches
struct Toggle : public Details::ActorBase
{
  static const Timestamp NO_CHANGE = 0;

  using Details::ActorBase::do_loop;

  Toggle(const byte pin)
    : _state(HIGH)
    , _tsChangeUsec(NO_CHANGE)
    , _pin(pin)
  {}
  void do_setup()
  {
    DEBUG("Switch setup");
    pinMode(_pin, INPUT_PULLUP);
  }
  bool isOn() const
  {
    return _state == LOW;
  }
  bool do_loop(const Timestamp now) override final
  {
    const byte newState = digitalRead(_pin);

    if(newState != _state)
    {
      // start debounce
      if(NO_CHANGE == _tsChangeUsec)
      {
        DEBUG("Switch state change");
        _tsChangeUsec = now;
      }
    }

    // if debounce in progress
    if(NO_CHANGE != _tsChangeUsec)
    {
      // check if debounce complete
      if(now - _tsChangeUsec >= Timeouts::TOGGLE_DEBOUNCE_DURATION_USEC)
      {
        _tsChangeUsec = NO_CHANGE;
        if(newState != _state)
        {
          _state = newState;
          if(isOn())
          {
            digitalWrite(LED_BUILTIN, HIGH);
            DEBUG("Switch ON.");
          }
          else
          {
            digitalWrite(LED_BUILTIN, LOW);
            DEBUG("Switch OFF.");
          }
        }
      }
    }
    // toggle is never done
    return false;
  }
  byte _state;
  Timestamp _tsChangeUsec;
  const byte _pin;
};

struct Buzzer : public Details::ActorBase
{
  friend struct Board;
  static const int NO_DELTA = 0;

  using Details::ActorBase::do_loop;

  Buzzer(const int rest, const int l1, const int l2, const char* nick)
    : _from(rest)
    , _to(rest)
    , _min(rest)
    , _max(rest)
    , _current(rest)
    , _delta(NO_DELTA)
    , _absMin(min(l1, l2))
    , _absMax(max(l1, l2))
    , _nick(nick)
  {
  }

  void do_setup(const byte pin)
  {
    DEBUG("Buzzer setup");
    pinMode(pin, OUTPUT);
    _servo.attach(pin);
    writeAngle();
  }

  bool do_loop(const unsigned long) override final
  {
    _current += _delta;
    if(_current < _min)
    {
      _current = _min;
    }
    else if(_current > _max)
    {
      _current = _max;
    }
    else
    {
      writeAngle();
    }
    return _current == _min or _current == _max;
  }

  void prepare(Timeline& timeline)
  {
    timeline.insert(micros(), this);
  }

  void prepare(const int from, const int to, Timestamp usec, Timeline& timeline)
  {
    prepare(timeline);
    prepare(from, to, usec);
  }

  void prepare(const int from, const int to, Timestamp usec)
  {
    setDelay(usec);
    prepare(from, to);
  }

  void next(const int to, Timestamp usec, Timeline& timeline)
  {
    DEBUG("'%s' next f=%d t=%d", _nick, _current, to);
    setDelay(usec);
    next(to, timeline);
  }

  void next(const int to, Timeline& timeline)
  {
    DEBUG("'%s' next f=%d t=%d", _nick, _current, to);
    prepare(timeline);
    prepare(_current, to);
  }

  void reverse(Timeline& timeline)
  {
    if(_delta != NO_DELTA)
    {
      Details::swap(_to, _from);
      _delta = -_delta;
      _current = _from;
      DEBUG("'%s' reverse f=%d t=%d c=%d d=%d mi=%d ma=%d", _nick, _from, _to, _current, _delta, _min, _max);

      prepare(timeline);
    }
  }

  void repeat()
  {
    if(_delta != NO_DELTA)
    {
      Details::swap(_from, _to);
      _delta = -_delta;
    }
  }

  //private:

  bool checkAndSetMinMax(const int from, const int to)
  {
    const int newMin = min(from, to);
    const int newMax = max(from, to);
    const bool result =
      (_absMin <= newMin and newMin <= _absMax
       and _absMin <= newMax and newMax <= _absMax);

    if(result)
    {
      _min = newMin;
      _max = newMax;
    }

    return result;
  }

  void writeAngle()
  {
    //DEBUG( "'%s' %d", _nick, _current );
    _servo.writeMicroseconds( _current );
  }

  void prepare(const int from, const int to)
  {
    _delta = NO_DELTA;

    if(from != to and checkAndSetMinMax(from, to))
    {
      _from = from;
      _to = to;
      _delta = ((_from < _to) ? (+1) : (-1));
      _current = _from;
      DEBUG("'%s' prepare f=%d t=%d c=%d d=%d mi=%d ma=%d", _nick, _from, _to, _current, _delta, _min, _max);
      writeAngle();
    }
  }

  int _from;
  int _to;
  int _min;
  int _max;

  int _current;
  int _delta;

  const int _absMin;
  const int _absMax;

  Servo _servo;
  const char* _nick;
};

struct Vibro : public Details::ActorBase
{
  Vibro(const byte pin)
    : _state(LOW)
    , _pin(pin)
    , _impulsesCount( 0 )
    , _impulseUsec( 0 )
    , _pauseUsec( 0 )
  {}
  void do_setup()
  {
    DEBUG("Vibro setup");
    pinMode(_pin, OUTPUT);
  }
  void off()
  {
    _state = LOW;
    _impulsesCount = 0;
    digitalWrite( _pin, _state );
  }
  void prepare(int impulsesCount, Timestamp impulseUsec, Timestamp pauseUsec, Timeline& timeline)
  {
    _impulsesCount = impulsesCount;
    _impulseUsec = impulseUsec;
    _pauseUsec = pauseUsec;
    _state = LOW;
    timeline.insert(micros(), this);
  }
  bool do_loop(const Timestamp now) override final
  {
    if( LOW == _state )
    {
      _state = HIGH;
      digitalWrite( _pin, _state );
      setDelay( _impulseUsec );
    }
    else if( HIGH == _state )
    {
      _state = LOW;
      digitalWrite( _pin, _state );
      --_impulsesCount;
      if( 0 == _impulsesCount )
      {
        return true;
      }
      else
      {
        setDelay( _pauseUsec );
      }
    }
    return false;
  }
  byte _state;
  const byte _pin;
  int _impulsesCount;
  Timestamp _impulseUsec;
  Timestamp _pauseUsec;
};
   
struct Piezo : public Details::ActorBase
{
  // there is very nice topic on notes
  // https://learn.digilentinc.com/Documents/392
  // but for sake of memory footprint reduction and simplicity
  // let's keep bare minimum
  
  Piezo( const byte pin )
    : _pin( pin )
    , _notes( nullptr )
    , _notesCount( 0 )
    , _noteIdx( 0 )
    , _wavesCount( 0 )
  {}
  
  void off()
  {
    digitalWrite( _pin, LOW );
  }

  void do_setup()
  {
    DEBUG( "Piezo setup" );
    pinMode( _pin, OUTPUT );
  }

  void prepare( Timeline& timeline, const char* melody )
  {
    if( nullptr != _notes )
    {
      _notesCount = 0;
      delete [] _notes;
    }
    _notes = Notes::prepare( melody, _notesCount );
    
    _noteIdx = -1;
    _wavesCount = 0;

    timeline.insert( micros(), this );
  }
  
  bool do_loop( const unsigned long ) override final
  {
    bool done = false;

    if( 0 == _wavesCount )
    {
      done = _noteIdx + 1 == _notesCount;
      if( not done )
      {
        ++_noteIdx;
        DEBUG( "next note %d: %lu %lu", _noteIdx, _notes[ _noteIdx ]._halfWaveUsec, _notes[ _noteIdx ]._count );
        setDelay( _notes[ _noteIdx ]._halfWaveUsec );
        _wavesCount = _notes[ _noteIdx ]._count;
        if( 0 == _wavesCount )
        {
          return done;
        }
      }
    }
    if( not done )
    {
      --_wavesCount;
      digitalWrite( _pin, _wavesCount % 2 == 1 ? HIGH : LOW );
      if( 0 == _wavesCount )
      {
        setDelay( 20000 );
      }
    }

    return done;
  }

  byte _pin;
  Notes::Note* _notes;
  int _notesCount;
  
  int _noteIdx;
  unsigned long _wavesCount;
};


struct Wait : public Details::ActorBase
{
  void prepare(const Timestamp usec, Timeline& timeline)
  {
    DEBUG("Wait prepare");
    timeline.insert(micros() + usec, this);
  }

  bool do_loop(const Timestamp now) override final
  {
    return true;
  }
};

struct Class
{
  Class()
    : toggle(Pins::Toggle)
    , servoArm(Angles::ArmRestDefault, Limits::ServoArmUpper, Limits::ServoArmLower, "The Arm")
    , servoCap(Angles::CapRestDefault, Limits::ServoCapUpper, Limits::ServoCapLower, "The Cap")
    , vibro(Pins::Vibro)
    , piezo(Pins::Piezo)
  {}
  Toggle toggle;
  Buzzer servoArm;
  Buzzer servoCap;
  Vibro vibro;
  Piezo piezo;
  Wait wait;
};

}

namespace Stages
{
  struct TimelineExecutorBase
  {
    virtual ~TimelineExecutorBase()
    {};
    virtual bool advance( Components::Class& components, Timeline& timeline, const Timestamp now ) = 0;
    virtual void prepare( Components::Class& components, Timeline& timeline ) = 0;

    TimelineExecutorBase( const char* nick )
      : _nick(nick)
    {}

    const char* _nick;
  };

  struct Stage : public TimelineExecutorBase
  {
    using PrepareFunc = void (*)(Components::Class&, Timeline&);
    
    Stage( const char* nick, PrepareFunc prepare )
      : TimelineExecutorBase( nick )
      , _prepare(prepare)
    {}
    
    void prepare( Components::Class& components, Timeline& timeline ) override final
    {
      DEBUG("Stage '%s' prepare", _nick);
      _prepare( components, timeline );
    }
    bool advance( Components::Class& components, Timeline& timeline, const Timestamp now ) override final
    {
      while(not timeline.empty())
      {
        //DEBUG( "advance enter now=%ld top=%ld", now, timeline.topVal() );
        if(timeline.topVal() <= now)
        {
          auto actor = timeline.topPayload();
          timeline.pop();
          actor->do_loop(now, timeline);
          continue;
        }
        //DEBUG( "advance break now=%ld", now );
        break;
      }
      return timeline.empty();
    }

    PrepareFunc _prepare;
  };

  struct StagePack : public TimelineExecutorBase
  {
    struct StageList
    {
      struct Node
      {
        Node(Node* prev, TimelineExecutorBase* stage)
          : _stage( stage )
          , _prev(prev)
          , _next(nullptr)
        {
        }
        TimelineExecutorBase* _stage;
        Node* _next;
        Node* _prev;
      };
  
      Node* _root = nullptr;
      Node* _tail = nullptr;
      Node* _current = nullptr;
  
      void addStage(TimelineExecutorBase* stage)
      {
        Node* node = new Node(_tail, stage);
        if(nullptr != _tail)
        {
          _tail->_next = node;
        }
        _tail = node;
        if(nullptr == _root)
        {
          _root = node;
        }
      }
      TimelineExecutorBase* reset()
      {
        _current = _root;
        return _current->_stage;
      }
      TimelineExecutorBase* getNextStage()
      {
        TimelineExecutorBase* out = nullptr;
        if(nullptr != _current and nullptr != _current->_next)
        {
          _current = _current->_next;
          out = _current->_stage;
        }
        return out;
      }
    };

    using AddStagesFunc = void (*)(StagePack*);

    StagePack( const char* nick, AddStagesFunc addStages )
      : TimelineExecutorBase( nick )
      , _currentStage( nullptr )
    {
      addStages( this );
    }

    template< class TType, class ... TArgs >
    void addStage( TArgs&& ... args )
    {
      _stages.addStage( new TType( args ... ) );
    }
  
    StageList _stages;
    TimelineExecutorBase* _currentStage;
  };

  struct SequentialPack : public StagePack
  {
    using StagePack::StagePack;
    
    void prepare( Components::Class& components, Timeline& timeline ) override final
    {
      _currentStage = _stages.reset();
      _currentStage->prepare( components, timeline );
    }

    bool advance( Components::Class& components, Timeline& timeline, const Timestamp now ) override
    {
      bool done = _currentStage->advance( components, timeline, now );
      if( done )
      {
        _currentStage = _stages.getNextStage();
        done = (nullptr == _currentStage);
        if( not done )
        {
          _currentStage->prepare( components, timeline );
        }
      }
      return done;
    }
  };

  struct ConcurrentPack : public StagePack
  {
    using StagePack::StagePack;
    
    void prepare( Components::Class& components, Timeline& timeline ) override final
    {
      auto it = _stages.reset();
      while( nullptr != it )
      {
        it->prepare( components, timeline );
        it = _stages.getNextStage();
      }
    }
    bool advance( Components::Class& components, Timeline& timeline, const Timestamp now ) override
    {
      bool done = true;
      auto it = _stages.reset();
      while( nullptr != it )
      {
        done = done and it->advance( components, timeline, now );
        it = _stages.getNextStage();
      }
      return done;
    }
  };
}

struct Scenario : public Stages::SequentialPack
{
  Scenario(const char* nick, Components::Class& components, AddStagesFunc addStages)
    : Stages::SequentialPack( nick, addStages )
    , _components( components )
    , _isToggleOff( false )
    , _isInterrupted( false )
  {
  }
  virtual ~Scenario() = default;
  bool isInterrupted() const
  {
    return _isInterrupted;
  }
  void reset()
  {
    DEBUG("Scenario '%s'", _nick);
    _isToggleOff = false;
    _isInterrupted = false;
    Stages::SequentialPack::prepare( _components, _timeline );
  }
  virtual bool isToggleSwitchedOnAgain()
  {
    bool isToggleOn = _components.toggle.isOn();
    bool out = _isToggleOff and isToggleOn;
    _isToggleOff = not isToggleOn;
    return out;
  }
  bool advance( const Timestamp now )
  {
    if(isToggleSwitchedOnAgain())
    {
      _timeline.drop();
      _isInterrupted = true;
      _components.vibro.off();
      _components.piezo.off();
      return _isInterrupted;
    }
    return Stages::SequentialPack::advance( _components, _timeline, now );
  }

  Timeline _timeline;
  Components::Class& _components;
  bool _isToggleOff;
  bool _isInterrupted;
};

// *INDENT-OFF*

namespace Scenarios
{

struct Toggle : public Scenario
{
  using Scenario::Scenario;
  bool isToggleSwitchedOnAgain() override final
  {
    return false;
  }
};

Scenario* createToggle( Components::Class& components )
{
  return new Toggle(
    "Toggle"
    , components
    , []( Stages::StagePack* scenario )
    {
      scenario->addStage< Stages::Stage >(
        
          "Toggle infinite stage"
          , [](Components::Class& components, Timeline& timeline)
          {
            components.toggle.setDelay(Timeouts::TOGGLE_DEBOUNCE_DURATION_USEC / 5);
            components.toggle.do_loop(micros(), timeline);
          }
        
      );
    }
  );
};

Scenario* createOrdinary( Components::Class& components )
{
  return new Scenario(
    "Ordinary"
    , components
    , []( Stages::StagePack* scenario )
    {
      scenario->addStage< Stages::Stage >(
        "Prepare Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.prepare(Angles::ArmRestDefault, Angles::ArmTargetDefault, Speeds::ArmDefault);
          components.wait.prepare(Timeouts::SERVO_POSITION_WAIT_USEC, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Open Cap"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoCap.prepare(Angles::CapRestDefault, Angles::CapTargetDefault, Speeds::CapDefault, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Hit Toggle"
        , [](Components::Class& components, Timeline& timeline)
        {
          // arm was prepared on the start
          components.servoArm.prepare(timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Wait Toggle switch"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.wait.prepare(Timeouts::TOGGLE_SWITCH_WAIT_USEC, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Hide Arm Under Cap"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmUnderCap, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Close Cap and hide Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmRestDefault, timeline);
          components.servoCap.next(Angles::CapRestDefault, timeline);
        }
      );
    }
  );
}

Scenario* createSlow(Components::Class& components)
{
  return new Scenario(
    "Slow", components
    , []( Stages::StagePack* scenario )
    {
      scenario->addStage< Stages::Stage >(
        "Prepare Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          // this scenario starts with arm position different from default
          // give servo some time to get there
          // note arm action is not added to timeline yet
          components.servoArm.prepare( Angles::ArmRestSlow, Angles::ArmTargetSlow, Speeds::ArmSlow );
          components.servoCap.prepare( Angles::CapRestDefault, Angles::CapTargetSlow, Speeds::CapSlow );
          // servo speed is 60 deg per 0.16 sec
          // travel between default position (where it supposed to be) is somewhat less than 60 deg
          components.wait.prepare(Timeouts::SERVO_POSITION_WAIT_USEC, timeline);
        }
      );
      scenario->addStage< Stages::ConcurrentPack >(
        "Triumph show up"
        , []( Stages::StagePack* scenario )
        {
          scenario->addStage< Stages::Stage >(
            "March"
            , [](Components::Class& components, Timeline& timeline)
            {
              components.piezo.prepare( timeline, Notes::dartIn );
            }
          );
          scenario->addStage< Stages::SequentialPack >(
            "Show and poke"
            , [](Stages::StagePack* scenario)
            {
              scenario->addStage< Stages::Stage >(
                "Open Cap ans show Arm slowly"
                , [](Components::Class& components, Timeline& timeline)
                {
                  components.servoCap.prepare(timeline);
                  components.servoArm.prepare(timeline);
                }
              );
              scenario->addStage< Stages::Stage >(
                "Poke around"
                , [](Components::Class& components, Timeline& timeline)
                {
                  components.wait.prepare(Timeouts::PAUSE_1_SEC_USEC, timeline);
                  components.vibro.prepare(1, Timeouts::PAUSE_1_SEC_USEC, 0, timeline);
                }
              );
            }
          );
        }
      );
      scenario->addStage< Stages::Stage >(
        "Hit Toggle"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmTargetDefault, Speeds::ArmDefault, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Wait Toggle switch"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.wait.prepare(Timeouts::TOGGLE_SWITCH_WAIT_USEC, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Drammatic pause"
        , [](Components::Class& components, Timeline& timeline)
        {
          // rollback Arm a bit to avoid servo overload
          components.servoArm.next(Angles::ArmTargetSwitchedOff, timeline);
          components.wait.prepare(Timeouts::PAUSE_3_SEC_USEC, timeline);
        }
      );

      scenario->addStage< Stages::ConcurrentPack >(
        "Triumph show down"
        , []( Stages::StagePack* scenario )
        {
          scenario->addStage< Stages::Stage >(
            "March"
            , [](Components::Class& components, Timeline& timeline)
            {
              components.piezo.prepare( timeline, Notes::dartOut );
            }
          );
          scenario->addStage< Stages::SequentialPack >(
            "Hide"
            , [](Stages::StagePack* scenario)
            {
              scenario->addStage< Stages::Stage >(
                "Open Cap ans show Arm slowly"
                , [](Components::Class& components, Timeline& timeline)
                {
                  components.servoArm.next(Angles::ArmRestSlow, Speeds::ArmSlow, timeline);
                  components.servoCap.next(Angles::CapRestDefault, timeline);
                }
              );
            }
          );
        }
      );
      scenario->addStage< Stages::Stage >(
        "Default Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmRestDefault, 0, timeline);
          components.wait.prepare(Timeouts::SERVO_POSITION_WAIT_USEC, timeline);
        }
      );
    }
  );
}

Scenario* createSwingArm(Components::Class& components)
{
  return new Scenario(
    "SwingArm", components
    , []( Stages::StagePack* scenario )
    {
      scenario->addStage< Stages::Stage >(
        "Prepare Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.prepare(Angles::ArmRestSlow, Angles::ArmRestSwing, Speeds::ArmDefault);
          // servo speed is 60 deg per 0.16 sec
          // travel between default position (where it supposed to be) is somewhat less than 60 deg
          components.wait.prepare(Timeouts::SERVO_POSITION_WAIT_USEC, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Open Cap"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoCap.prepare(Angles::CapRestDefault, Angles::CapTargetSwing, Speeds::CapDefault, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Ready for swing poisition"
        , [](Components::Class& components, Timeline& timeline)
        {
          // arm was prepared on the start
          components.servoArm.prepare(timeline);
        }
      );
      for(int i = 0 ; i < 3 ; ++i)
      {
        scenario->addStage< Stages::Stage >(
          "Swing"
          , [](Components::Class& components, Timeline& timeline)
          {
            components.servoArm.next(Angles::ArmTargetSwitchedOn, Speeds::ArmDefault, timeline);
          }
        );
        scenario->addStage< Stages::Stage >(
          "Pause between swings"
          , [](Components::Class& components, Timeline& timeline)
          {
            components.wait.prepare(Timeouts::PAUSE_1_SEC_USEC, timeline);
          }
        );
        scenario->addStage< Stages::Stage >(
          "Rollback"
          , [](Components::Class& components, Timeline& timeline)
          {
            components.servoArm.next(Angles::ArmRestSwing, Speeds::ArmSlow, timeline);
            components.vibro.prepare(1, Timeouts::PAUSE_1_SEC_USEC, 0, timeline);
          }
        );
      }
      scenario->addStage< Stages::Stage >(
        "Rollback event further"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmRestSwingLast, Speeds::ArmSlow, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Hit Toggle"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmTargetDefault, Speeds::ArmRapid, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Wait Toggle switch"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.wait.prepare(Timeouts::TOGGLE_SWITCH_WAIT_USEC, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Hide Arm Under Cap"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.piezo.prepare( timeline, Notes::no );
          components.servoArm.next(Angles::ArmUnderCap, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Close Cap and hide Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmRestDefault, timeline);
          components.servoCap.next(Angles::CapRestDefault, timeline);
        }
      );
    }
  );
}

Scenario* createFlapCap(Components::Class& components)
{
  return new Scenario(
    "FlapCap", components
    , []( Stages::StagePack* scenario )
    {
      scenario->addStage< Stages::Stage >(
        "Prepare Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.prepare(Angles::ArmRestSlow, Angles::ArmTargetDefault, Speeds::ArmDefault);
          components.servoCap.next(Angles::CapRestDefault, Speeds::CapFlap, timeline);
          // servo speed is 60 deg per 0.16 sec
          // travel between default position (where it supposed to be) is somewhat less than 60 deg
          components.wait.prepare(Timeouts::SERVO_POSITION_WAIT_USEC, timeline);
        }
      );
      for(int i = 0 ; i < 3 ; ++i)
      {
        scenario->addStage< Stages::Stage >(
          "Open Cap"
          , [](Components::Class& components, Timeline& timeline)
          {
            components.servoCap.next(Angles::CapTargetDefault, timeline);
          }
        );
        scenario->addStage< Stages::Stage >(
          "Close Cap"
          , [](Components::Class& components, Timeline& timeline)
          {
            components.servoCap.reverse(timeline);
          }
        );
        scenario->addStage< Stages::Stage >(
          "Pause between flaps"
          , [](Components::Class& components, Timeline& timeline)
          {
            components.wait.prepare(Timeouts::PAUSE_3OO_MSEC_USEC, timeline);
            components.vibro.prepare(1, Timeouts::PAUSE_2OO_MSEC_USEC, Timeouts::PAUSE_1OO_MSEC_USEC, timeline);
          }
        );
      }
      scenario->addStage< Stages::Stage >(
        "Pause after flaps"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.wait.prepare(Timeouts::PAUSE_3_SEC_USEC, timeline);
        }
      );
    }
  );
}

Scenario* createPokeAround(Components::Class& components)
{
  return new Scenario(
    "PokeAround", components
    , []( Stages::StagePack* scenario )
    {
      scenario->addStage< Stages::Stage >(
        "Prepare Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.prepare(Angles::ArmRestSlow, Angles::ArmTargetPokeAround, Speeds::ArmDefault);
          // servo speed is 60 deg per 0.16 sec
          // travel between default position (where it supposed to be) is somewhat less than 60 deg
          components.wait.prepare(Timeouts::SERVO_POSITION_WAIT_USEC, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Open Cap"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoCap.prepare(Angles::CapRestDefault, Angles::CapTargetDefault, Speeds::CapSlow, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Poke around"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.prepare(timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Drammatic pause"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.piezo.prepare( timeline, Notes::no );
          components.wait.prepare(Timeouts::PAUSE_3_SEC_USEC, timeline);
          components.vibro.prepare(10, Timeouts::PAUSE_2OO_MSEC_USEC, Timeouts::PAUSE_1OO_MSEC_USEC, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Hide Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.reverse(timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Close Cap"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoCap.reverse(timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Another drammatic pause"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmRestDefault, timeline);
          components.wait.prepare(Timeouts::PAUSE_3_SEC_USEC, timeline);
        }
      );
    }
  );
}

Scenario* createNoShow(Components::Class& components)
{
  return new Scenario(
    "NoShow", components
    , []( Stages::StagePack* scenario )
    {
      scenario->addStage< Stages::Stage >(
        "Prepare Arm"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.prepare(Angles::ArmRestSlow, Angles::ArmUnderCap, Speeds::ArmSlow);
          // servo speed is 60 deg per 0.16 sec
          // travel between default position (where it supposed to be) is somewhat less than 60 deg
          components.wait.prepare(Timeouts::SERVO_POSITION_WAIT_USEC, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Move Arm under Cap"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.prepare(timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Move Arm back"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmRestDefault, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Drammatic pause"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.wait.prepare(Timeouts::PAUSE_1O_SEC_USEC, timeline);
        }
      );
    }
  );
}

Scenario* createUninterrupter(Components::Class& components)
{
  return new Scenario(
    "Uninterrupter", components
    , []( Stages::StagePack* scenario )
    {
      scenario->addStage< Stages::Stage >(
        "Force Cap open"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoCap.next(Angles::CapTargetSwing, Speeds::CapDefault, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Hit Toggle"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmTargetDefault, Speeds::ArmRapid, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Convincing pause"
        , [](Components::Class& components, Timeline& timeline)
        {
          // rollback Arm a bit to avoid servo overload
          components.servoArm.next(Angles::ArmTargetSwitchedOff, timeline);
          components.piezo.prepare( timeline, Notes::no );
          components.vibro.prepare(1, Timeouts::PAUSE_1_SEC_USEC, 0, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Move Arm under Cap"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmUnderCap, timeline);
        }
      );
      scenario->addStage< Stages::Stage >(
        "Close"
        , [](Components::Class& components, Timeline& timeline)
        {
          components.servoArm.next(Angles::ArmRestDefault, timeline);
          components.servoCap.next(Angles::CapRestDefault, timeline);
        }
      );
    }
  );
}

Scenario* createDebug( Components::Class& components )
{
  return new Scenario(
      "Debug"
      , components
      , []( Stages::StagePack* )
      {
      }
  );
}

}

// *INDENT-ON*

struct Board : public Components::Class
{
    //enum ScenarioId : byte
    enum ScenarioId
    {
      UNINTERRUPTER = 0
      , ORDINARY
      , SWINGARM
      , FLAPCAP
      , SLOW
      , POKEAROUND
      , NOSHOW
      , COUNT_OF_SCENARIOS
    };

    Board()
      : _mainSequenceId( ORDINARY )
      , _scenarioToggle(Scenarios::createToggle(*this))
      , _scenario(0)
      , _scenarioDebug(Scenarios::createDebug(*this))
    {
    }
    ~Board()
    {
      for(int i = 0 ; i < COUNT_OF_SCENARIOS; ++i)
      {
        delete _scenarios[ i ];
      }
    }
    void do_setup()
    {
      Serial.begin(115200);
      
      pinMode(LED_BUILTIN, OUTPUT);

      toggle.do_setup();
      servoArm.do_setup(Pins::Arm);
      servoCap.do_setup(Pins::Cap);
      piezo.do_setup();
      vibro.do_setup();

      _scenarioToggle->reset();

      _scenarios[ UNINTERRUPTER ] = Scenarios::createUninterrupter(*this);
      _scenarios[ ORDINARY ] = Scenarios::createOrdinary( *this );
      _scenarios[ SLOW ] = Scenarios::createSlow(*this);
      _scenarios[ SWINGARM ] = Scenarios::createSwingArm(*this);
      _scenarios[ FLAPCAP ] = Scenarios::createFlapCap(*this);
      _scenarios[ POKEAROUND ] = Scenarios::createPokeAround(*this);
      _scenarios[ NOSHOW ] = Scenarios::createNoShow(*this);

      randomSeed(analogRead(0));
    }

    void do_loop()
    {
      Timestamp now = micros();
      _scenarioToggle->advance(now);

      if(nullptr != _scenario)
      {
        // advance scenario
        if(_scenario->advance(now))
        {
          if(_scenario->isInterrupted())
          {
            pickScenario(UNINTERRUPTER);
          }
          // scenarios which do not toggle switch itself
          //   flap cap
          //   poke around
          //   no show
          else if(_scenario == _scenarios[ FLAPCAP ]
                  or _scenario == _scenarios[ POKEAROUND ]
                  or _scenario == _scenarios[ NOSHOW ]
                 )
          {
            // so run ordinary scenario after it
            pickScenario(ORDINARY);
          }
          else
          {
            // done
            _scenario = nullptr;
            DEBUG("scenario done");
          }
        }
      }
      else if( toggle.isOn() )
      {
        pickScenario();
      }
      else if(Serial.available() > 0)
      {
        // cap target
        char id = Serial.read();
        Serial.read();
        long param = Serial.parseInt();
        Serial.read();
        if(id == 'b')
        {
          unsigned long f = param;
          param = Serial.parseInt();
          Serial.read();
          unsigned long msec = param;
          unsigned long waveDurationUsec = 1000000. / f;
          unsigned long wavesCount = ( msec * 1000 ) / waveDurationUsec;
          DEBUG( "f %ld msec %ld usec %ld wc %ld", f, msec, waveDurationUsec, wavesCount );
          DEBUG( "hw %ld", (waveDurationUsec >> 1) );
          for( unsigned long i = 0 ; i < wavesCount ; ++i )
          {
            digitalWrite( Pins::Piezo, HIGH );
            delayMicroseconds( (waveDurationUsec >> 1) );
            digitalWrite( Pins::Piezo, LOW );
            delayMicroseconds( (waveDurationUsec >> 1) );
          }
          return;
        }
        DEBUG("%c %d", id, param);
        if( id == 'v' )
        {
          digitalWrite( Pins::Vibro, HIGH );
          delay(param);
          digitalWrite( Pins::Vibro, LOW );
          return;
        }
        if((id == 'a' or id == 'c') and 544 <= param and param <= 2400)
        {
          Components::Buzzer* obj = 0;
          switch(id)
          {
            default:
              break;
            case 'c':
              {
                obj = &servoCap;
              }
              break;
            case 'a':
              {
                obj = &servoArm;
              }
          }
          if(obj != 0)
          {
            obj->_servo.writeMicroseconds(param);
          }
        }
        if(id == 's')
        {
          DEBUG("HIT THE TOGGLE NOW");
          delay(3000);
          _scenarioToggle->advance(micros());
          delay(100);
          _scenarioToggle->advance(micros());
          if(param < ORDINARY or COUNT_OF_SCENARIOS <= param)
          {
            param = ORDINARY;
          }
          pickScenario(param);
        }
      }
    }

  private:

    void pickScenario(byte id)
    {
      _scenario = _scenarios[ id ];
      _scenario->reset();
    }

    void pickScenario()
    {
      byte id = _mainSequenceId++;
      if(id >= COUNT_OF_SCENARIOS)
      {
        id = random(ORDINARY, COUNT_OF_SCENARIOS);
      }
      pickScenario(id);
    }

    byte _mainSequenceId;

    Scenario* _scenarioToggle;
    Scenario* _scenario;
    Scenario* _scenarios[ COUNT_OF_SCENARIOS ];
    Scenario* _scenarioDebug;
};

//### main loop

Board board;

void setup()
{
  board.do_setup();
}

void loop()
{
  board.do_loop();
}

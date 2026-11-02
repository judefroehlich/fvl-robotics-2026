# Code Explained

A line-by-line walkthrough of the drivetrain control code: what every piece
does, why the C++ looks the way it does, and where the math comes from.

Written for teammates who can read basic C++ (loops, functions, `if`) but have
not had to deal with namespaces, references, templates, or lambdas yet. The
harder language features each get a full explanation the first time they show
up, and there is a crash-course section up front you can jump back to.

Files covered, in the order it makes sense to read them:

| File                                                                                | What lives there                                               |
| ----------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| [`include/pid.hpp`](include/pid.hpp) / [`src/pid.cpp`](src/pid.cpp)                 | The PID controller and the "has it stopped yet" helper         |
| [`include/profile.hpp`](include/profile.hpp) / [`src/profile.cpp`](src/profile.cpp) | The trapezoidal motion profile                                 |
| [`include/chassis.hpp`](include/chassis.hpp) / [`src/chassis.cpp`](src/chassis.cpp) | Glues them to real motors and the inertial sensor              |
| [`src/main.cpp`](src/main.cpp)                                                      | Ports, physical measurements, gains, and the PROS entry points |

---

## 1. The big picture

The job is: _"drive forward 24 inches and stop there accurately."_

A naive version is "run the motors at half power until the encoder says 24
inches, then stop." That fails in two ways. It slams into full power
instantly, which makes the wheels slip and the robot tip, and it coasts past
the target because the robot has momentum when it cuts power.

The fix splits the problem into two independent halves:

```
                    where it SHOULD be              what voltage to send
                      at this instant                   to get there
                           |                                 |
  drive_distance(24)       v                                 v
         |          TrapezoidProfile  --> error -->  PID + feedforward --> motors
         |                  ^                                 ^
         |                  |                                 |
         +-- builds it      |                            measured position
                            |                              (encoders)
                      a smooth ramp:
                  speed up, cruise, slow down
```

- **The profile is the plan.** It answers "at 0.3 seconds into this move,
  where should the robot be, how fast should it be going, and how hard should
  it be accelerating?" It never looks at the robot at all — it is pure math,
  decided before the move starts.
- **The PID is the correction.** Every 10 milliseconds it compares where the
  robot _should_ be to where it _is_, and nudges the voltage to close the gap.
- **The feedforward is the educated guess.** Before the PID even gets
  involved, we already know roughly what voltage produces a given speed. We
  send that immediately, so the PID only has to clean up the leftovers instead
  of discovering the whole voltage from scratch.

This split is the single most important idea in the codebase. A PID _alone_
has to see an error before it reacts, so it is always playing catch-up. A
profile _alone_ has no idea the robot is lagging. Together, the feedforward
does ~90% of the work, the profile keeps the motion smooth, and the PID
handles the last few percent.

---

## 2. C++ crash course

Everything in this section shows up later. Skim it now, come back when a
symbol confuses you.

### 2.1 Headers, `.cpp` files, and translation units

C++ compiles one `.cpp` file at a time, completely independently. Each one,
with all its `#include`s pasted in, is called a **translation unit**. The
compiler turns each into a `.o` object file, and then the _linker_ staples
them together into one program.

`#include "pid.hpp"` is a literal copy-paste performed by the preprocessor.
That is all it is.

This is why we split declarations from definitions:

- **The header (`.hpp`)** says _what exists_: "there is a class called `PID`,
  it has a method `update` that takes two doubles and returns a double."
- **The source (`.cpp`)** says _how it works_: the actual body of `update`.

Anyone who wants to _use_ `PID` includes the header. Only one `.cpp` contains
the body. If the body lived in the header and two files included it, the
linker would find two copies of `PID::update` and refuse to continue
("multiple definition").

### 2.2 Include guards

```cpp
#ifndef _FVL_PID_HPP_      // if this name has NOT been defined yet
#define _FVL_PID_HPP_      // define it now
...                        // the whole file body
#endif
```

`chassis.hpp` includes `pid.hpp`, and `main.h` includes both. Without the
guard, `pid.hpp`'s contents would be pasted in twice and you would get
"redefinition of class PID". The guard makes the second paste expand to
nothing. Three lines of boilerplate at the top and bottom of every header.

(`#pragma once` does the same thing in one line and most compilers support it.
The PROS headers use it. Ours use the classic guards; either is fine, just be
consistent within a file.)

### 2.3 Namespaces — the "namespace shit"

A namespace is a named box you put things in to stop name collisions. Our code
lives in `namespace fvl`:

```cpp
namespace fvl {
class PID { ... };
}  // namespace fvl
```

The class's real, full name is now `fvl::PID`. The `::` is the **scope
resolution operator** — read it as "inside". `fvl::PID` is "the `PID` inside
`fvl`". `std::fabs` is "the `fabs` inside `std`".

Why bother? Because `PID` is an obvious name that a library might also use. If
PROS ever shipped a `PID` class, and both were in the global namespace, the
compiler could not tell them apart. `fvl::PID` and `pros::PID` coexist fine.

Three things to know:

**Inside the namespace, you can drop the prefix.** In `pid.cpp` everything is
wrapped in `namespace fvl { ... }`, which is why `clamp(...)` works there
without writing `fvl::clamp(...)`. From outside — like `main.cpp` — you need
the full `fvl::Chassis`.

**The closing comment is a convention, not syntax.** `}  // namespace fvl`
exists because a closing brace 80 lines later is otherwise meaningless to a
human. The compiler ignores it.

**`using namespace` dumps everything into the current scope.** `main.h` offers
`using namespace pros;` as a commented-out option, which would let you write
`Motor` instead of `pros::Motor`. Convenient in a `.cpp`, a genuinely bad idea
in a header, because every file that includes it silently inherits the dump.
We do not use it.

### 2.4 The anonymous namespace

```cpp
namespace {          // <-- no name
constexpr std::uint32_t kLoopMs = 10;
double wrap_180(double degrees) { ... }
}  // namespace
```

A namespace with no name means **"private to this file."** Everything inside
gets _internal linkage_: the linker never sees it, so another `.cpp` cannot
refer to it, and two different `.cpp` files can each have their own
`wrap_180` without colliding.

We use this twice:

- In `chassis.cpp`, for the helpers (`kLoopMs`, `wrap_180`, `follow_profile`)
  that are implementation details nobody outside should touch.
- In `main.cpp`, for the robot globals (`left_drive`, `chassis`, ...) so they
  are file-private rather than truly global.

You may see older code use the `static` keyword for the same effect. The
anonymous namespace is the modern way and works on types too, which `static`
does not.

### 2.5 `pros::v5` and why we `#include` instead of forward declaring

This one bit me while writing the code, so it is worth knowing.

A **forward declaration** is a promise that a type exists without saying what
is in it:

```cpp
namespace pros { class MotorGroup; }   // "trust me, this exists"
```

That is normally the lightweight way to reference a type in a header — it
compiles faster because you skip the whole real header. It works as long as
you only use references or pointers to the type, not its members.

It does **not** work here. PROS actually declares its devices like this:

```cpp
namespace pros {
inline namespace v5 {          // <-- note "inline"
class MotorGroup : public virtual AbstractMotor { ... };
}
}
```

An **inline namespace** is a versioning trick: `MotorGroup` really lives in
`pros::v5`, but `inline` makes it _also_ visible as `pros::MotorGroup`, so
existing code keeps working if PROS later adds a `v6`.

The catch: if we forward declare `class MotorGroup;` directly inside
`namespace pros`, we are not naming their class. We are declaring a
**brand-new, different** class that happens to share a name. The compiler then
has two candidates for `pros::MotorGroup` and errors out — or worse, silently
picks ours and the linker fails later with a missing symbol.

So `chassis.hpp` includes the real headers:

```cpp
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
```

**Rule of thumb:** forward declaring someone else's type is only safe if you
know they did not put it in an inline namespace, behind a typedef, or behind a
template alias. Including the header always works. Do that unless compile time
is actually hurting.

### 2.6 References (`&`) versus pointers (`*`)

This is the concept that trips people up most, so here it is in full.

A **reference** is another name for an existing object. Not a copy — the same
object:

```cpp
int x = 5;
int& r = x;     // r IS x, under a different name
r = 10;         // x is now 10
```

Three rules: a reference must be initialized when created, it can never be
pointed at a different object afterwards, and it can never be null.

A **pointer** stores a memory address. It is a separate variable holding a
number that happens to be where something lives:

```cpp
int x = 5;
int* p = &x;    // p holds the ADDRESS of x
*p = 10;        // follow the address and write there; x is now 10
p = nullptr;    // legal: p now points at nothing
```

Pointers can be reseated, can be null, and need `*` to read through
("dereference").

**`&` means two completely different things depending on where it appears:**

| Where                                      | Meaning               | Example            |
| ------------------------------------------ | --------------------- | ------------------ |
| In a **declaration** (next to a type)      | "this is a reference" | `pros::Imu& imu_;` |
| In an **expression** (in front of a value) | "address of"          | `&loop_stamp`      |

Same symbol, unrelated jobs. The only way to tell is context: is it part of a
type, or applied to a value?

**Where we use references.** `Chassis` stores three:

```cpp
pros::MotorGroup& left_;
pros::MotorGroup& right_;
pros::Imu& imu_;
```

Note the `&`. Without it, `Chassis` would hold _copies_ of the motor group
objects. Copies would be a disaster: they would be separate device handles,
and writing to the copy would not reliably be the same thing the rest of the
program is reading. We want `Chassis` to talk to _the_ motors.

They are references rather than pointers because there is no such thing as a
chassis with no motors. A pointer would imply "this might be null, check it
first," and we would have to write that check everywhere. A reference makes
"always present" a compile-time guarantee.

**Where we use an actual pointer.** Exactly one place, `chassis.cpp:87`:

```cpp
pros::Task::delay_until(&loop_stamp, kLoopMs);
```

`delay_until` needs to _modify_ `loop_stamp` (it advances it by 10 ms each
call), and PROS declares that parameter as a pointer:

```cpp
static void delay_until(std::uint32_t* const prev_time, const std::uint32_t delta);
```

So `&loop_stamp` is the address-of operator, handing over the variable's
address so the function can write through it. If PROS had used a reference
parameter we would just pass `loop_stamp` and the modification would still
happen — references are generally the nicer choice for this, but PROS exposes
a C-compatible API here.

### 2.7 Constructors and member initializer lists

```cpp
Chassis::Chassis(pros::MotorGroup &left, pros::MotorGroup &right,
                 pros::Imu &imu, ChassisConfig config)
    : left_(left), right_(right), imu_(imu), config_(config) {}
```

The part after the `:` and before the `{}` is the **member initializer list**.
It initializes members _before_ the constructor body runs.

For reference members this is not a style choice, it is the only option.
References must be bound at creation, so there is no way to write this in the
body:

```cpp
Chassis::Chassis(...) {
    left_ = left;   // ERROR: this assigns THROUGH the reference,
                    // it does not bind it. And left_ was never bound.
}
```

The body `{}` is empty because the initializer list already did everything.

Note the trailing underscore on `left_`, `config_`, `integral_`. That is a
naming convention marking a private member, which keeps the constructor
parameter `left` and the member `left_` visually distinct. Nothing enforces
it; it is just a habit that avoids confusion.

### 2.8 `struct` versus `class`

Technically the only difference is the default access level: `struct` members
are public by default, `class` members are private.

By convention we use `struct` for plain bags of data with no behavior
(`PIDGains`, `ProfileState`, `ChassisConfig`) and `class` for things with
logic and hidden state (`PID`, `TrapezoidProfile`, `Chassis`).

### 2.9 Default member initializers and designated initializers

Struct members can carry defaults:

```cpp
struct PIDGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double integral_zone = 0.0;
};
```

So `PIDGains g;` gives you all zeros rather than four garbage values. **An
uninitialized `double` in C++ contains whatever bits were in that memory** —
not zero. Defaults eliminate a whole category of bug.

Then you can set members by name, which is a C++20 feature called
**designated initializers**:

```cpp
.pid = {.kP = 1400.0, .kI = 800.0, .kD = 200.0, .integral_zone = 3.0},
```

Compare to the positional form `{1400.0, 800.0, 200.0, 3.0}` — the named
version is self-documenting and, more importantly, cannot silently break if
someone reorders the struct's fields. Anything you leave out keeps its
default. This works because the project builds with `gnu++26` (set in
[`common.mk`](common.mk)).

### 2.10 `const` in its various positions

`const` means "cannot be modified," but _what_ it protects depends on where it
sits.

```cpp
const double start_heading = heading();   // this variable never changes
```

```cpp
void follow_profile(const TrapezoidProfile &profile, ...)
```

A **const reference parameter**. Two jobs: it avoids copying the object (a
reference), and it promises the function will not modify it (`const`). This is
the standard way to pass anything bigger than a couple of numbers.

```cpp
double position() const;
```

A **const member function**. The `const` goes _after_ the parameter list and
promises this method does not modify the object. You can call it on a `const
Chassis`, and inside it every member becomes const.

There is a subtlety worth knowing: inside `position() const`, the member
`left_` is a _const reference_, which means **the reference itself** cannot be
reseated — but references can never be reseated anyway, so the `const` adds
nothing, and the `MotorGroup` it refers to is **not** const. Calling a
non-const method on `left_` from a const member function compiles fine. Const
does not propagate through references or pointers. That is occasionally
surprising and occasionally useful.

### 2.11 `constexpr`

```cpp
constexpr double kMaxVoltage = 12000.0;
constexpr std::uint32_t kLoopMs = 10;
```

"Compute this at compile time and treat it as a true constant." The value is
baked into the instructions; there is no variable in memory and no runtime
lookup. Use it for named constants instead of `#define`, because unlike a
preprocessor macro it has a real type and respects scope.

The `k` prefix (`kMaxVoltage`, `kLoopMs`) is a convention marking a compile-time
constant, so you can tell at a glance that it will not change under you.

### 2.12 `static_cast`

```cpp
left_.move_voltage(static_cast<std::int32_t>(clamp(left_mv, kMaxVoltage)));
```

An explicit type conversion. `clamp` returns a `double` but `move_voltage`
wants a 32-bit integer, so we convert. Going from `double` to `int` **truncates
toward zero** — 11999.7 becomes 11999, and -0.8 becomes 0.

You could write `(std::int32_t)x` in C style. `static_cast` is preferred
because it is searchable, and because it refuses conversions that are actually
unsafe instead of silently reinterpreting memory the way a C cast can.

### 2.13 Templates and lambdas

These two appear together in `follow_profile`, so they are explained in
context in [section 5.4](#54-follow_profile--the-control-loop). The short
version:

- A **lambda** is an anonymous function you can define inline and that can
  capture surrounding variables: `[this] { return position(); }`.
- A **template** is a function the compiler stamps out once per set of types
  it is used with, so it works with any type that supports the right
  operations.

### 2.14 Unsigned integer arithmetic

`std::uint32_t` is an unsigned 32-bit integer: range 0 to 4,294,967,295, no
negatives. `std::int32_t` is the signed version. `std::int8_t` is a signed
8-bit integer, -128 to 127.

The important property: **unsigned arithmetic wraps around cleanly** and is
defined to work modulo 2³². This is relied on in `chassis.cpp:56`:

```cpp
const std::uint32_t elapsed = pros::millis() - start;
```

`pros::millis()` counts milliseconds since the program started, in a
`uint32_t`, so it rolls over to 0 after about 49.7 days. If that rollover
happened mid-move, `millis()` would be a tiny number and `start` would be
huge — the subtraction looks like it should go negative and break.

It does not. Modular arithmetic gives the right answer anyway. If
`start = 4294967290` and `millis() = 4`, then `4 - 4294967290` wraps to `10`,
which is the correct elapsed time. You get this for free as long as you
subtract _unsigned_ values and never store the intermediate in a signed type.

### 2.15 Integer versus floating-point division

A trap that silently produces zero:

```cpp
constexpr double kLoopSeconds = kLoopMs / 1000.0;   // 0.01   correct
//                              kLoopMs / 1000      // 0      WRONG
```

`kLoopMs` is an integer. `kLoopMs / 1000` is integer division: 10/1000 = 0,
remainder discarded. Writing `1000.0` forces the whole expression to double.

Same reasoning at `chassis.cpp:60`, `profile.at(elapsed / 1000.0)` — `elapsed`
is a `uint32_t`, and the `.0` is what keeps it from collapsing to whole
seconds.

### 2.16 `extern "C"` in main.h

```cpp
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
...
#endif
```

C++ **mangles** function names — it encodes the parameter types into the
symbol, so `autonomous()` might become `_Z10autonomousv` in the object file.
That is how overloading works: two functions with the same name but different
parameters get different symbols.

The PROS kernel is C, and it calls `autonomous` by that exact spelling.
`extern "C"` says "do not mangle these; use the plain name." Without it the
linker would never match them up.

The `#ifdef __cplusplus` around it is because `extern "C"` is not valid C, and
this header has to be includable from both languages.

---

## 3. `pid.hpp` / `pid.cpp` — the controller

### 3.1 What PID actually computes

PID looks at the **error** — how far off target you are — and produces a
control effort. Three terms, each answering a different question:

$$u(t) = k_P \, e(t) \;+\; k_I \int_0^t e(\tau)\,d\tau \;+\; k_D \frac{de(t)}{dt}$$

where $e = \text{target} - \text{measured}$, and $u$ is the output (for us,
millivolts).

- **P — proportional. "How far off am I right now?"** Push harder when further
  away. This is the main term. On its own it leaves a permanent error, because
  as the error shrinks so does the push, until it is too weak to move the
  robot at all.
- **I — integral. "How long have I been off?"** Accumulates error over time,
  so a small error that refuses to go away keeps building pressure until it
  does. This is what kills the steady-state error P leaves behind.
- **D — derivative. "How fast is the error changing?"** Error shrinking fast
  means we are about to overshoot, so D pushes back. It is the brakes, and it
  is what damps oscillation.

Nobody computes a real integral or derivative on a robot. Over one 10 ms
loop, with `dt` the time since the last call:

$$\text{integral} \mathrel{+}= e \cdot dt \qquad\qquad \text{derivative} = \frac{e - e_{\text{prev}}}{dt}$$

The integral is a rectangle-sum (Riemann sum) approximation; the derivative is
a finite difference. At 100 Hz, that is plenty accurate.

### 3.2 `PIDGains` — the three constants plus one

```cpp
struct PIDGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double integral_zone = 0.0;
};
```

Grouping the gains in a struct rather than passing three loose doubles means
you cannot accidentally swap `kI` and `kD` at a call site, and the whole set
can be stored and copied as one value.

`integral_zone` is explained in 3.6.

**The gains carry the units.** This is worth internalizing. `kP` is not a
dimensionless number — it has units of _millivolts per inch_ for the drive and
_millivolts per degree_ for turns. That is why `kP = 1400` for driving and
`kP = 400` for turning are both reasonable: an inch of position error is a
much bigger deal than a degree of heading error, and the gain absorbs the
conversion. It is also why you cannot copy gains between robots with different
gearing.

### 3.3 The class declaration

```cpp
class PID {
   public:
    explicit PID(PIDGains gains, double output_limit = kMaxVoltage);
    double update(double error, double dt);
    void reset();
    void set_gains(PIDGains gains) { gains_ = gains; }
    PIDGains gains() const { return gains_; }

   private:
    PIDGains gains_;
    double output_limit_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool have_prev_ = false;
};
```

`public:` is the interface anyone can use; `private:` is internal state only
the class touches. The point of the split is that `integral_` has an invariant
to maintain (it must stay clamped), so outside code must not be able to poke
it directly.

`explicit` blocks implicit conversion. Without it, this would compile:

```cpp
PIDGains g{...};
PID p = g;        // silently constructs a PID from gains
```

which is nonsense that reads like a typo. `explicit` forces `PID p(g);`.

`double output_limit = kMaxVoltage` is a **default argument** — write
`PID(gains)` and it fills in 12000. Defaults must be trailing parameters,
since the compiler matches arguments left to right.

`set_gains` and `gains()` are defined inline in the header because they are
one-liners; the compiler can then paste them in at the call site rather than
emitting a real function call. Note `gains()` is `const` (reading does not
modify) while `set_gains` is not.

`have_prev_` tracks whether we have seen an error yet, which matters on the
very first call — see 3.7.

### 3.4 `clamp`

```cpp
double clamp(double value, double limit) {
  limit = std::fabs(limit);
  if (value > limit)
    return limit;
  if (value < -limit)
    return -limit;
  return value;
}
```

Squashes `value` into `[-limit, limit]`. `std::fabs` is floating-point
absolute value (`abs` is the integer one; using it on a double would truncate).

Taking `fabs(limit)` first means a caller passing a negative limit gets the
sensible symmetric range rather than an impossible one where every value is
simultaneously too high and too low.

Note `limit` is modified inside the function. That is safe — it is a
by-value parameter, so it is a local copy and the caller's variable is
untouched.

### 3.5 The constructor

```cpp
PID::PID(PIDGains gains, double output_limit)
    : gains_(gains), output_limit_(std::fabs(output_limit)) {}
```

`PID::PID` means "the constructor of `PID`, which lives in namespace `fvl`
because this whole file is wrapped in it." The `::` here is the same
scope-resolution operator from 2.3, used to say which class this definition
belongs to.

Default arguments appear in the _declaration_ (the header), never repeated in
the definition. Writing them in both places is a compile error.

`integral_`, `prev_error_`, and `have_prev_` are not in the list because their
default member initializers in the header already handle them.

### 3.6 `update`, step by step

```cpp
double PID::update(double error, double dt) {
  double derivative = 0.0;

  if (dt > 0.0) {
```

Everything time-based is guarded by `dt > 0.0`. Division by zero on a double
does not crash — it yields infinity, which then contaminates every later
calculation as `NaN` and makes the motors do something undefined. Guarding
once here is cheaper than hunting that down later. With `dt <= 0` the function
degrades to proportional-only, which is a safe fallback.

```cpp
    // Sign flip means we just crossed the setpoint; dumping the integral
    // here is what keeps the overshoot from being paid back as a wobble.
    if (error * prev_error_ < 0.0)
      integral_ = 0.0;
```

Multiplying the current and previous error is a sign test: the product is
negative only if exactly one of them is negative, meaning the error changed
sign and we just passed through the target.

Why dump the integral there? Because everything it accumulated was pressure
built up to get us _to_ the target. Now that we are past it, that stored push
is pointing the wrong way, and working it back off takes time — the robot
sails past, slowly unwinds, comes back, overshoots the other way. Clearing it
at the crossing removes the stored energy exactly when it stops being useful.

```cpp
    bool in_zone =
        gains_.integral_zone <= 0.0 || std::fabs(error) < gains_.integral_zone;
    if (in_zone)
      integral_ += error * dt;
```

The **integral zone**. Only accumulate when the error is already small.

Here is the problem it solves. On a 48-inch move the error starts at 48
inches. If we integrate from the start, by the time we arrive the integral
holds an enormous sum from the long approach, and that demands full voltage
long after we should have stopped. The integral's job is grinding out the last
fraction of an inch, not helping with the bulk of the move — the P term and
the feedforward handle that. So we switch it on only inside the zone (3 inches
for drives, 8 degrees for turns).

`gains_.integral_zone <= 0.0 ||` makes zero mean "no zone, always integrate."
The `||` **short-circuits**: if the left side is true, the right side is never
evaluated. Here that is just a minor efficiency, but the same property is what
makes `if (p != nullptr && p->value)` safe.

```cpp
    // Clamp the integral so it can never demand more than the full output
    // on its own.
    if (gains_.kI != 0.0) {
      integral_ = clamp(integral_, output_limit_ / std::fabs(gains_.kI));
    } else {
      integral_ = 0.0;
    }
```

**Anti-windup.** Suppose the robot is jammed against a wall. The error never
shrinks, so the integral grows without bound. Free the robot and it launches
at full power for however long it takes to unwind that number.

The fix bounds the integral so its _contribution_ can never exceed the total
output limit. We want $|k_I \cdot \text{integral}| \le \text{limit}$, so
rearranged, $|\text{integral}| \le \text{limit} / |k_I|$. That is exactly the
clamp.

The `else` branch zeroes the integral when `kI` is zero. Without it, the
integral would quietly pile up while contributing nothing, and the moment
someone called `set_gains` with a nonzero `kI` mid-motion, that garbage would
hit the motors all at once.

```cpp
    if (have_prev_)
      derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    have_prev_ = true;
  }
```

The finite-difference derivative, skipped on the first call. See 3.7.

```cpp
  double output =
      gains_.kP * error + gains_.kI * integral_ + gains_.kD * derivative;
  return clamp(output, output_limit_);
}
```

Sum the three terms, clamp to the output limit. The final clamp matters
because the three terms are computed independently and can easily add up past
12000 mV; sending that to `move_voltage` would just be silently capped by the
motor, but clamping here keeps our own numbers honest.

### 3.7 Why `have_prev_` exists: derivative kick

On the very first `update` call, `prev_error_` is 0.0 — not because the error
was zero a moment ago, but because nothing has happened yet.

If we computed the derivative anyway, on a 24-inch move we would get:

$$\frac{e - e_{\text{prev}}}{dt} = \frac{24 - 0}{0.01} = 2400 \text{ in/s}$$

Multiply by `kD = 200` and the D term alone demands 480,000 mV. Clamped to
12000, so the robot slams to full reverse on the first loop of every move.
That artifact is called **derivative kick**, and `have_prev_` is the one-line
fix: no previous sample, no derivative.

### 3.8 A nice consequence of differentiating the error

Textbooks often warn that taking the derivative of the _error_ is worse than
taking it of the _measurement_, because a sudden setpoint change produces a
kick. That warning does not apply to us, and it is worth understanding why.

Our error is $e = x_{\text{target}} - x_{\text{measured}}$, so:

$$\frac{de}{dt} = v_{\text{target}} - v_{\text{measured}}$$

Because the setpoint comes from a smooth profile rather than a step, the D
term is not reacting to a jump — it is a **velocity tracking error**. It
measures "am I moving at the speed the plan calls for," and pushes to fix the
difference. For a profiled follower that is precisely what you want, and it is
a real benefit of feeding a PID from a profile instead of a constant target.

### 3.9 Why P alone always stops short: the friction deadband

This is the most practically important piece of math in the file, and it is
why `kI` is not optional on this robot.

A drivetrain does not move at all until the applied voltage overcomes static
friction — gearbox drag, bearing stiction, the weight of the robot. Call that
threshold $V_f$. Measured on a typical V5 drive it is somewhere around
800–1100 mV.

A pure-P controller outputs $k_P \cdot e$. It stops producing motion when:

$$k_P \cdot e < V_f \quad \Longrightarrow \quad e < \frac{V_f}{k_P}$$

So the robot parks at a steady error of about $V_f / k_P$ and sits there
forever. The error is nonzero, P dutifully commands a voltage, and the robot
does not care because the voltage is below the threshold.

Plug in numbers:

|                 | $V_f$   | $k_P$ | parks at      |
| --------------- | ------- | ----- | ------------- |
| Turn, low gain  | 1100 mV | 50    | **22°** short |
| Turn, our gain  | 1100 mV | 400   | 2.75° short   |
| Drive, our gain | 900 mV  | 1400  | 0.64 in short |

This is why my first attempt at "safe conservative gains" performed _worst_ in
simulation — they never settled at all, because the deadband error was larger
than the tolerance the motion was waiting for. There is a hard **lower** bound
on usable gain, which is the opposite of the usual "start low and creep up"
advice.

Two ways out, and we use both:

1. **Raise `kP`** so $V_f/k_P$ drops under the tolerance.
2. **Add `kI`.** The integral does not care how small the error is; it keeps
   accumulating until the output crosses $V_f$ and the robot finally budges.
   This is the robust fix, and it is what the integral term is _for_.

### 3.10 `SettleConfig` and `Settler`

Knowing when a move is finished is its own small problem. "Error is zero" is
useless — noise alone means it is never exactly zero, and the robot passes
through zero at full speed mid-overshoot. The test needs to be "close enough,
slow enough, for long enough."

```cpp
struct SettleConfig {
    double tolerance = 1.0;
    double velocity_tolerance = 0.0;
    std::uint32_t settle_time_ms = 150;
};
```

```cpp
bool Settler::update(double error, double velocity, std::uint32_t dt_ms) {
  bool close = std::fabs(error) <= config_.tolerance;
  bool slow = config_.velocity_tolerance <= 0.0 ||
              std::fabs(velocity) <= config_.velocity_tolerance;

  if (close && slow) {
    settled_ms_ += dt_ms;
  } else {
    settled_ms_ = 0;
  }

  return settled_ms_ >= config_.settle_time_ms;
}
```

A stopwatch that only runs while both conditions hold, and **resets to zero
the instant either fails**. That reset is the whole design: it means
`settled_ms_` measures a _continuous_ stretch of being settled, so a robot
coasting through the tolerance band at speed cannot accumulate credit across
multiple passes.

With `dt_ms = 10` and `settle_time_ms = 150`, it takes 15 consecutive good
loops to return true.

As with `integral_zone`, `velocity_tolerance <= 0.0` means "ignore this
check," so you can opt out and test position only.

The `Settler` constructor is defined inline in the header:

```cpp
explicit Settler(SettleConfig config) : config_(config) {}
```

Same member-initializer-list pattern as `PID`, just short enough to live in
the header.

---

## 4. `profile.hpp` / `profile.cpp` — the motion profile

### 4.1 The idea

A **trapezoidal profile** is a speed plan with three phases: ramp up at
constant acceleration, hold a constant cruise speed, ramp down at constant
deceleration. Graph velocity against time and you get a trapezoid, which is
where the name comes from.

```
velocity
   ^
   |        ______________________
   |       /                      \
   |      /                        \
   |     /                          \
   |    /                            \
   +---+------------------------------+------> time
       |<-t_accel->|<-cruise->|<-t_accel->|
       0                              t_total

       \__________ area = distance __________/
```

The key insight that makes all the math work: **position is the area under
the velocity curve.** That is just what integration means — if you are
travelling at velocity $v$ for time $t$, you cover $v \cdot t$, and summing
that over a changing velocity is the integral. So "how far have I gone by time
$t$" is always "what is the area of the velocity graph up to $t$."

That gives us the two formulas everything else is built from. Starting from
rest at constant acceleration $a$:

$$v(t) = a t \qquad\qquad x(t) = \tfrac{1}{2} a t^2$$

The second is the area of a triangle with base $t$ and height $at$:
$\frac{1}{2} \cdot t \cdot at = \frac{1}{2}at^2$.

### 4.2 Why bother, versus just running the PID at a fixed target?

If you hand a PID the final target directly, the error starts huge, so the P
term immediately demands full voltage. The wheels slip, the robot lurches, and
the only thing limiting acceleration is how much traction you happen to have.

With a profile, the _commanded_ position moves smoothly, so the error stays
small throughout and the voltage stays reasonable. Acceleration becomes a
number you choose rather than an accident. You also get a known duration up
front, which is what lets us compute a sane default timeout.

### 4.3 The header

```cpp
#ifndef _FVL_PROFILE_HPP_
#define _FVL_PROFILE_HPP_

namespace fvl {
```

Note there are **no `#include`s at all** in this header. It uses only `double`
and needs nothing from the standard library. `profile.cpp` includes `<cmath>`
for `sqrt` and `fabs`, but that is an implementation detail and does not
belong in the header. Keeping headers free of unnecessary includes is a habit
worth having — every include is pasted into every file that uses it.

```cpp
struct ProfileConstraints {
    double max_velocity = 0.0;      // units per second
    double max_acceleration = 0.0;  // units per second squared
};

struct ProfileState {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};
```

"Units" is deliberately vague, and that is the point — this class does not
know or care whether it is planning inches or degrees. Feed it inches and
in/s² and you get inches back. Feed it degrees and you get degrees. The same
code drives both `drive_distance` and `turn_by`.

`ProfileState` bundles three return values into one. C++ functions return a
single value, and a struct is the clean way to hand back a group.

```cpp
    double duration() const { return t_total_; }
    bool is_done(double t) const { return t >= t_total_; }
    double distance() const { return distance_; }
    double cruise_velocity() const { return cruise_velocity_; }
```

Four one-line **accessors**, all `const`, all defined in the header so they
compile down to a direct field read with no function-call overhead. The
members stay private so nobody outside can corrupt the relationships between
them (`t_total_` must always equal `t_cruise_end_ + t_accel_`, and so on).

```cpp
   private:
    double distance_ = 0.0;          // signed
    double direction_ = 1.0;         // +1 or -1
    double cruise_velocity_ = 0.0;   // unsigned
    double acceleration_ = 0.0;      // unsigned
    double accel_distance_ = 0.0;    // unsigned, covered during the ramp up
    double t_accel_ = 0.0;           // end of the ramp up
    double t_cruise_end_ = 0.0;      // start of the ramp down
    double t_total_ = 0.0;
```

The `signed` / `unsigned` comments are doing real work here. The whole class
computes the profile for a **forward** move using magnitudes only, then
applies the direction at the very end. Mixing the two conventions up is how
you get a profile that works perfectly forwards and drives into a wall
backwards, so each field is labelled with which one it uses.

### 4.4 The constructor: planning the move

Everything is decided once, up front. `at()` later just evaluates formulas.

```cpp
TrapezoidProfile::TrapezoidProfile(double distance,
                                   ProfileConstraints constraints)
    : distance_(distance) {
  direction_ = distance < 0.0 ? -1.0 : 1.0;
```

`condition ? a : b` is the **ternary operator**, a compact if/else that
produces a value. Here: negative distance gives -1.0, anything else gives
+1.0. (Zero gives +1.0, which is harmless since a zero-distance profile goes
down the degenerate path anyway.)

`distance_` is set in the initializer list; `direction_` in the body. Either
works for a plain `double` — only references and const members are forced into
the list.

```cpp
  double d = std::fabs(distance);
  double v_max = std::fabs(constraints.max_velocity);
  double a_max = std::fabs(constraints.max_acceleration);
```

Strip all the signs immediately. From here on we are planning a forward move
of length `d`. Taking `fabs` of the constraints too means a caller who
mistakenly passes a negative acceleration gets sane behavior instead of a
profile that runs backwards in time.

```cpp
  if (d == 0.0 || v_max == 0.0 || a_max == 0.0) {
    cruise_velocity_ = 0.0;
    acceleration_ = 0.0;
    accel_distance_ = 0.0;
    t_accel_ = t_cruise_end_ = t_total_ = 0.0;
    return;
  }
```

The degenerate guard. A zero distance, or a zero velocity/acceleration budget,
would mean dividing by zero below. Instead we build a zero-length profile.

`a = b = c = 0.0` is a **chained assignment**. Assignment in C++ is an
expression that evaluates to the assigned value, and it groups right to left,
so this reads as `t_accel_ = (t_cruise_end_ = (t_total_ = 0.0))`.

What a zero-length profile _means_ downstream matters: `at()` will report the
full distance as the commanded position immediately, and `is_done()` is
instantly true. So the control loop degenerates gracefully into a plain PID
move straight at the target. Not great, not a crash.

```cpp
  acceleration_ = a_max;

  // Distance needed to ramp up to v_max and back down again.
  double triangle_distance = v_max * v_max / a_max;
```

**Where $v^2/a$ comes from.** Reaching $v_{max}$ from rest takes
$t_a = v_{max}/a$, and covers:

$$x_{\text{ramp}} = \tfrac{1}{2} a t_a^2 = \tfrac{1}{2} a \left(\frac{v_{max}}{a}\right)^2 = \frac{v_{max}^2}{2a}$$

Slowing back down is the mirror image, so it covers the same distance again:

$$d_{\text{both ramps}} = 2 \cdot \frac{v_{max}^2}{2a} = \frac{v_{max}^2}{a}$$

That is the shortest move that still touches full speed. Anything longer has
room to cruise; anything shorter cannot get there.

```cpp
  if (d < triangle_distance) {
    // Too short to ever reach v_max: peak where the two ramps meet.
    cruise_velocity_ = std::sqrt(a_max * d);
    t_accel_ = cruise_velocity_ / a_max;
    accel_distance_ = d / 2.0;
    t_cruise_end_ = t_accel_;
```

**The triangular case.** The move is too short to reach the speed cap, so the
profile is accelerate-then-immediately-decelerate, peaking in the middle.

By symmetry each half covers $d/2$. Finding the peak speed $v_p$:

$$\frac{d}{2} = \frac{v_p^2}{2a} \quad\Longrightarrow\quad v_p^2 = a d \quad\Longrightarrow\quad v_p = \sqrt{a d}$$

which is the `std::sqrt(a_max * d)`. This is the same algebra as the standard
$v^2 = v_0^2 + 2a\Delta x$ with $v_0 = 0$ and $\Delta x = d/2$.

Setting `t_cruise_end_ = t_accel_` gives the cruise phase **zero length**.
That is the trick that lets one set of formulas cover both shapes — a triangle
is just a trapezoid whose flat top has been squeezed to nothing. In `at()`,
the cruise branch tests `t < t_cruise_end_`, which can never be true once
`t >= t_accel_`, so the branch is simply skipped.

(`accel_distance_` is set to `d/2` for consistency but is never actually read
in this case, since the only place it is used is that unreachable cruise
branch.)

```cpp
  } else {
    cruise_velocity_ = v_max;
    t_accel_ = v_max / a_max;
    accel_distance_ = v_max * v_max / (2.0 * a_max);
    double cruise_distance = d - 2.0 * accel_distance_;
    t_cruise_end_ = t_accel_ + cruise_distance / v_max;
  }
```

**The full trapezoid.** We do reach the cap, so cruise speed is `v_max` and
the ramp time is $v_{max}/a$. Subtracting both ramps from the total leaves the
cruise distance, and at constant speed, time is just distance over speed.

```cpp
  t_total_ = t_cruise_end_ + t_accel_;
}
```

Total time is "when deceleration starts" plus "how long deceleration takes" —
and deceleration is the mirror of acceleration, so it takes the same
`t_accel_`. This one line is correct for both shapes: in the triangular case
`t_cruise_end_ == t_accel_`, so it gives $2 t_{accel}$.

Worth noting the closed forms, which are handy for sanity checks:

$$t_{\text{total}} = \begin{cases} 2\sqrt{d/a} & \text{triangular} \\[4pt] \dfrac{v_{max}}{a} + \dfrac{d}{v_{max}} & \text{trapezoidal} \end{cases}$$

The trapezoidal one falls out of $2\frac{v}{a} + \frac{d - v^2/a}{v}$ after
cancelling. For a 24-inch move at 28 in/s and 60 in/s²:
$28/60 + 24/28 = 1.324$ s.

### 4.5 `at()`: evaluating the plan

```cpp
ProfileState TrapezoidProfile::at(double t) const {
  ProfileState state;
```

`ProfileState state;` is all zeros, thanks to the default member initializers
in the struct. Without those this would be three garbage values and every
branch below would have to remember to set all three fields.

`at()` is `const` — asking the profile a question never changes it. The profile
is immutable once built, which is what makes it safe to call from anywhere.

```cpp
  if (t_total_ <= 0.0) {
    state.position = distance_;
    return state;
  }

  if (t <= 0.0) {
    return state; // still at the start, at rest
  }

  if (t >= t_total_) {
    state.position = distance_;
    return state;
  }
```

Three guard clauses, all returning early:

- **Degenerate profile** — hand back the full displacement immediately.
- **Before the start** — at the origin, at rest. The all-zero `state` is
  already correct, so there is nothing to assign.
- **After the end** — parked at the target, at rest.

That last one matters more than it looks. The control loop keeps running after
the profile finishes, while the settler waits out its 150 ms. During that
window `at()` keeps returning "you should be exactly at the target, not
moving," which is exactly the setpoint you want for holding position.

These three use `distance_` (already signed) and **return before** the
direction multiply at the bottom. That is deliberate: applying `direction_` to
an already-signed value would flip reverse moves back to forwards. Easy bug to
introduce if you ever add a fourth guard clause here — the early `return` is
load-bearing.

```cpp
  if (t < t_accel_) {
    // Ramping up.
    state.position = 0.5 * acceleration_ * t * t;
    state.velocity = acceleration_ * t;
    state.acceleration = acceleration_;
```

Straight from $x = \frac{1}{2}at^2$ and $v = at$. Note `t * t` rather than
`std::pow(t, 2)` — for squaring, multiplication is faster and exactly as
clear.

```cpp
  } else if (t < t_cruise_end_) {
    // Cruising at the velocity cap.
    state.position = accel_distance_ + cruise_velocity_ * (t - t_accel_);
    state.velocity = cruise_velocity_;
    state.acceleration = 0.0;
```

Position is "everything covered during the ramp" plus "constant speed times
time spent cruising so far." `(t - t_accel_)` converts absolute time into time
_since cruise began_, which is the quantity the formula needs. Acceleration is
zero because speed is constant.

```cpp
  } else {
    // Ramping down. Mirror of the ramp up, measured back from the end.
    double remaining = t_total_ - t;
    state.position =
        std::fabs(distance_) - 0.5 * acceleration_ * remaining * remaining;
    state.velocity = acceleration_ * remaining;
    state.acceleration = -acceleration_;
  }
```

**The deceleration phase, and the neatest trick in the file.**

You could derive this the hard way, tracking position from the start through
all three phases. Instead, look at it backwards. Run time in reverse from the
end: the robot starts at the target with zero speed and accelerates away from
it. That is _identical_ to the ramp-up case, just measured from the other end.

So with `remaining` = time left until the move ends:

- Speed is $a \cdot \text{remaining}$ — large at the start of the ramp-down,
  reaching zero exactly at $t_{total}$.
- Distance still to cover is $\frac{1}{2}a \cdot \text{remaining}^2$, so
  position is total minus that.
- Acceleration is $-a$, the one place the sign is explicit, because we are
  slowing down.

`std::fabs(distance_)` because this branch works in magnitudes like the rest
of the constructor; the sign is applied below.

It is worth checking continuity at the handover, $t = t_{\text{cruise end}}$,
where `remaining` equals $t_{accel} = v/a$:

- Velocity: $a \cdot \frac{v}{a} = v$ ✓ matches the cruise speed.
- Position: $d - \frac{1}{2}a\left(\frac{v}{a}\right)^2 = d - \frac{v^2}{2a} = d - x_{\text{ramp}}$,
  and the cruise branch at that same instant gives
  $x_{\text{ramp}} + (d - 2x_{\text{ramp}}) = d - x_{\text{ramp}}$ ✓.

No jump in either, which is what keeps the commanded setpoint smooth and stops
the D term from spiking at the transition.

```cpp
  state.position *= direction_;
  state.velocity *= direction_;
  state.acceleration *= direction_;
  return state;
}
```

The direction applied once, at the end, to all three. This is why the rest of
the class could ignore sign entirely. A reverse move gets a negative position,
a negative velocity, and a negative acceleration during the ramp-up — which is
correct, since "accelerating backwards" is negative acceleration.

### 4.6 Verifying it

The profile math is pure arithmetic with no hardware involved, which makes it
testable on a laptop. The checks that were run:

- **Integrate the velocity numerically and compare to the position.** They
  must agree, since position is by definition the integral of velocity. This
  catches any inconsistency between the three formulas, and would have caught
  a wrong coefficient anywhere.
- **End conditions** — position equals distance and velocity equals zero at
  `t_total`.
- **The constraints are respected** — peak velocity never exceeds
  `max_velocity`, peak acceleration never exceeds `max_acceleration`.
- **Both shapes, both directions**, plus the exact boundary where
  $d = v^2/a$ and the two cases meet (no discontinuity there).
- **Degenerate inputs** — zero distance, zero constraints, negative
  constraints, times before 0 and past the end. None produce `NaN` or hang.
---

## 5. `chassis.hpp` / `chassis.cpp` — wiring it to the robot

This is where the abstract pieces meet real motors. It is the densest file, and
it contains the two hardest language features in the project (templates and
lambdas) plus the physics of the feedforward.

### 5.1 The configuration structs

```cpp
struct ChassisConfig {
    double wheel_diameter = 3.25;
    double gear_ratio = 1.0;
    double track_width = 11.5;
};
```

Physical measurements. `gear_ratio` is **wheel revolutions per motor
revolution**, and getting it backwards is a classic mistake that scales every
distance by the square of your error.

Through a gear mesh, teeth times rotations is conserved:

$$N_{\text{motor}} \cdot \theta_{\text{motor}} = N_{\text{wheel}} \cdot \theta_{\text{wheel}} \quad\Longrightarrow\quad \frac{\theta_{\text{wheel}}}{\theta_{\text{motor}}} = \frac{N_{\text{motor}}}{N_{\text{wheel}}}$$

So a 36-tooth gear on the motor driving a 48-tooth gear on the wheel is
`36.0 / 48.0` = 0.75, not 48/36. The wheel turns *slower* than the motor,
which is the whole point of gearing down for torque. Had we written 1.333
instead, every commanded distance would come out 1.78× too long.

`track_width` is not used by `drive_distance` or `turn_by` — both measure
rotation from the inertial sensor rather than deriving it from wheel
positions. It is there for when someone adds arc/swing turns, which do need
it.

```cpp
struct FeedForward {
    double kS = 0.0;
    double kV = 0.0;
    double kA = 0.0;
};
```

The physics constants. See 5.5.

```cpp
struct MotionTuning {
    PIDGains pid;
    FeedForward ff;
    ProfileConstraints constraints;
    SettleConfig settle;
};
```

A struct of structs: everything tunable about one *kind* of motion. The
drivetrain needs two complete independent sets — one for inches, one for
degrees — and bundling them means `follow_profile` can take a single
`MotionTuning` parameter instead of a dozen loose doubles.

### 5.2 The `Chassis` class

```cpp
class Chassis {
   public:
    Chassis(pros::MotorGroup& left, pros::MotorGroup& right, pros::Imu& imu, ChassisConfig config);

    MotionTuning drive_tuning;
    MotionTuning turn_tuning;
    PIDGains heading_hold;
```

Note the tuning structs are **public data members**, not hidden behind
setters. That is a deliberate tradeoff: they have no invariants to protect
(any combination of gains is "valid," just possibly bad), and leaving them
public means tuning from `main.cpp` is a plain assignment. A setter per field
would be a lot of ceremony for nothing.

```cpp
   private:
    double degrees_to_inches(double motor_degrees) const;

    pros::MotorGroup& left_;
    pros::MotorGroup& right_;
    pros::Imu& imu_;
    ChassisConfig config_;
};
```

The three references from [2.6](#26-references--versus-pointers). `config_` is
stored **by value** — it is three doubles, copying is free, and the chassis
should own its own snapshot rather than depend on the caller keeping the
original alive. The references are references because device handles must be
shared, not copied.

### 5.3 Small member functions

```cpp
Chassis::Chassis(pros::MotorGroup &left, pros::MotorGroup &right,
                 pros::Imu &imu, ChassisConfig config)
    : left_(left), right_(right), imu_(imu), config_(config) {}
```

Member initializer list, mandatory for the references. Empty body.

```cpp
double Chassis::degrees_to_inches(double motor_degrees) const {
  return motor_degrees / 360.0 * config_.gear_ratio * M_PI *
         config_.wheel_diameter;
}
```

**The unit conversion**, read left to right as three successive steps:

$$\underbrace{\frac{\theta_{\text{motor}}}{360}}_{\text{motor revs}} \times \underbrace{\text{gear\_ratio}}_{\text{wheel revs}} \times \underbrace{\pi \, d}_{\text{inches per wheel rev}} = \text{inches}$$

1. Motor degrees over 360 gives motor revolutions.
2. Times `gear_ratio` (wheel revs per motor rev) gives wheel revolutions.
3. Times the circumference $\pi d$ gives linear travel.

`M_PI` comes from `<cmath>`. It is a POSIX constant rather than strictly
standard C++, but GCC provides it, which is all we need here.

This assumes the wheel rolls without slipping. It does slip under hard
acceleration, which is one more reason to cap `max_acceleration` rather than
let the PID demand whatever it likes.

```cpp
double Chassis::position() const {
  return degrees_to_inches((left_.get_position() + right_.get_position()) /
                           2.0);
}
```

Averaging the two sides. For a straight move both read nearly the same and the
average is the travel. The average also **cancels rotation**: during a turn in
place one side goes forward exactly as much as the other goes back, so the
average stays near zero. That is a useful property, and it is exactly why
turns are measured from the inertial sensor instead of from this.

```cpp
double Chassis::heading() const { return imu_.get_rotation(); }
```

`get_rotation()` is **continuous and unbounded** — turn clockwise four full
circles and it reads 1440, not 0. Clockwise is positive.

This is the right choice and the alternative is a trap. PROS also offers
`get_heading()`, which wraps to [0, 360). With that, `turn_by(720)` would be
impossible to express and any turn crossing the 0/360 seam would compute a
nonsense error. Unbounded rotation makes `heading() - start_heading` just work,
no matter how far you turn.

```cpp
void Chassis::reset_position() {
  left_.tare_position_all();
  right_.tare_position_all();
}
```

Zeroes the encoders. `tare_position_all` does every motor in the group; the
plain `tare_position` would only do one.

```cpp
void Chassis::calibrate() {
  imu_.reset(true);
  while (imu_.is_calibrating())
    pros::delay(kLoopMs);
  imu_.tare_rotation();
}
```

The inertial sensor needs a couple of seconds on boot to measure its gyro
bias, and reports garbage until it finishes. `reset(true)` — the `true` means
blocking — should already wait, and the `while` loop is belt-and-braces in
case it returns early. `tare_rotation()` then defines "wherever the robot is
now" as zero.

The `pros::delay(kLoopMs)` inside the loop matters: a bare `while
(is_calibrating());` would spin the CPU at 100% and starve every other task.
Always yield inside a wait loop.

```cpp
void Chassis::set_voltage(double left_mv, double right_mv) {
  left_.move_voltage(static_cast<std::int32_t>(clamp(left_mv, kMaxVoltage)));
  right_.move_voltage(static_cast<std::int32_t>(clamp(right_mv, kMaxVoltage)));
}
```

Clamp, then convert to the integer type the API wants. The clamp is defensive —
`move_voltage` would saturate internally anyway — but it keeps our own numbers
truthful, which matters if you ever print them while debugging.

**Two different motor APIs, and why.** PROS gives you both:

| Call | Range | Meaning |
|---|---|---|
| `move(x)` | -127 … 127 | Legacy scale, matches raw joystick values |
| `move_voltage(mv)` | -12000 … 12000 | Millivolts, a real physical unit |

Control code uses `move_voltage` because the feedforward constants are
expressed in millivolts — `kV` is "millivolts per inch per second," which is
only meaningful if we actually command millivolts. Driver control uses `move`
because joystick axes are already -127…127 and no conversion is needed.

```cpp
void Chassis::stop() {
  left_.brake();
  right_.brake();
}
```

`brake()` applies whichever brake mode was configured (`main.cpp` sets COAST).
It is not the same as `move_voltage(0)`: COAST lets the robot roll freely,
while HOLD actively drives back toward the stopped position.

### 5.4 `follow_profile` — the control loop

The heart of the file. One function runs every profiled motion; `drive_distance`
and `turn_by` differ only in what they measure and how they apply the output.

```cpp
template <typename Measure, typename Apply>
void follow_profile(const TrapezoidProfile &profile, const MotionTuning &tuning,
                    Measure measure, Apply apply, std::uint32_t timeout_ms) {
```

**Templates.** `template <typename Measure, typename Apply>` says "`Measure`
and `Apply` are placeholders; fill them in with whatever types the caller
passes." The compiler then generates a *separate copy* of this function for
each distinct combination it sees — one for the drive call, one for the turn
call.

Why that matters here: the alternative is `std::function<double()>`, which
would also accept any callable but does so through a type-erased indirect
call, usually with a heap allocation. On a microcontroller running a 100 Hz
loop, we would rather not. With a template the compiler knows the exact
callable at compile time and **inlines it**, so `measure()` compiles down to
the same instructions as if `position()` had been written there literally.
Zero overhead for the flexibility.

The cost is that templates must be *defined* where they are used, which is why
`follow_profile` lives in the `.cpp` (both callers are in the same file)
rather than the header. If another file needed it, the definition would have
to move to the header.

```cpp
  PID pid(tuning.pid);
  Settler settler(tuning.settle);
```

Both constructed fresh for every motion, so no integral or settle-timer state
leaks from the previous move. This is why `PID::reset()` is rarely needed in
practice — we get a clean one each time.

```cpp
  // Never loop forever: a robot pinned against a wall would otherwise hang
  // the whole autonomous routine.
  if (timeout_ms == 0) {
    timeout_ms = static_cast<std::uint32_t>(profile.duration() * 2000.0) + 1500;
  }
```

A safety net. Without it, `drive_distance(24)` with no explicit timeout would
loop forever if the robot got stuck — and in a match that hangs the entire
autonomous routine, not just this one motion.

Since the profile knows its own duration, a sensible bound is twice that plus
1.5 s of settling margin. `duration()` is in seconds, so `* 2000.0` converts
to milliseconds *and* doubles it in one step.

Note we are assigning to `timeout_ms`, a **by-value parameter** — a local copy,
so the caller's variable is untouched. Same situation as `limit` in `clamp`.

```cpp
  const std::uint32_t start = pros::millis();
  std::uint32_t loop_stamp = start;
  double previous = measure();
  double velocity = 0.0;
```

`start` is the reference time for the whole motion. `loop_stamp` is separate
because `delay_until` will modify it. `previous` primes the velocity estimate
with the starting position so the first difference is not spurious.

```cpp
  while (true) {
    const std::uint32_t elapsed = pros::millis() - start;
    if (timeout_ms > 0 && elapsed >= timeout_ms)
      break;
```

Unsigned subtraction, correct across rollover, as in [2.14](#214-unsigned-integer-arithmetic).

The `timeout_ms > 0 &&` is dead code — the block above guarantees
`timeout_ms >= 1500`. Harmless, and it documents the intent, but it can never
be false.

```cpp
    const ProfileState target = profile.at(elapsed / 1000.0);
    const double current = measure();
```

Ask the plan where we should be, ask the robot where it is. `elapsed / 1000.0`
converts ms to seconds — and the `.0` is essential, per
[2.15](#215-integer-versus-floating-point-division).

`measure()` calls through the template parameter. For a drive it is
`position()`; for a turn it is `heading() - start_heading`.

```cpp
    velocity +=
        kVelocityFilter * ((current - previous) / kLoopSeconds - velocity);
    previous = current;
```

**The velocity estimate, and the bug that made it necessary.**

The raw estimate is a finite difference: $(x_n - x_{n-1}) / dt$. Over a 10 ms
window that is almost pure noise. The inertial sensor jitters about ±0.15°
even when perfectly still, and dividing that by 0.01 s gives **±30 °/s of
completely fictional velocity.**

That broke the settle check outright. With `velocity_tolerance` at 5 °/s, the
condition could essentially never be satisfied, so *every* turn would have run
to its timeout instead of settling — looking like a tuning problem while
actually being a measurement problem. The simulation caught it before it ever
reached the robot.

The fix is an **exponential moving average**. Expanding the `+=`:

$$v_n = v_{n-1} + \alpha\left(v_{\text{raw}} - v_{n-1}\right) = (1-\alpha)\,v_{n-1} + \alpha\, v_{\text{raw}}$$

So each new estimate is 92% of the old one plus 8% of the new measurement. It
is a low-pass filter in one line, with no buffer and no history beyond a
single double.

With $\alpha = 0.08$:

- **Noise** drops by $\sqrt{\alpha/(2-\alpha)} = \sqrt{0.08/1.92} \approx 0.20$,
  so roughly a 5× reduction — the ±30 °/s becomes about ±6 °/s.
- **Lag** is a time constant of about $dt/\alpha = 0.01/0.08 = 125$ ms.

That lag is the tradeoff, and it is why this value is only used for the settle
check and never fed back into the controller. A 125 ms delay inside a feedback
loop is a great way to cause oscillation; a 125 ms delay on a "have we stopped
yet" test is harmless.

```cpp
    double output = pid.update(target.position - current, kLoopSeconds);
```

The error is **commanded position minus actual position** — against the
profile's instantaneous setpoint, not the final target. This is the core of
profiled control: we are asking "am I where the plan says I should be *right
now*," which keeps the error small all the way through.

`kLoopSeconds` is passed as `dt` rather than a measured elapsed time. That is
accurate because `delay_until` holds the loop to a true 10 ms. If the loop body
ever overran its budget the real `dt` would be larger and the D and I terms
would be slightly off — worth knowing, not worth fixing at this scale.

```cpp
    output +=
        tuning.ff.kV * target.velocity + tuning.ff.kA * target.acceleration;
    if (target.velocity > 0.0) {
      output += tuning.ff.kS;
    } else if (target.velocity < 0.0) {
      output -= tuning.ff.kS;
    }
```

The feedforward, added on top of the PID correction. Full physics in 5.5.

Note `kS` keys off the **target** velocity's sign, not the measured one. Using
the measured sign would chatter: near zero the sign flips with every noise
sample, slamming ±800 mV back and forth. The commanded velocity is smooth and
noise-free, so it is the better reference. And when the target velocity is
exactly zero — before the start, after the end — no `kS` is applied at all,
which is what lets the robot actually come to rest instead of creeping.

```cpp
    apply(clamp(output, kMaxVoltage));
```

Clamp to the 12 V the battery can supply, then hand off to the caller's
lambda to split across the two sides.

```cpp
    if (profile.is_done(elapsed / 1000.0) &&
        settler.update(profile.distance() - current, velocity, kLoopMs)) {
      break;
    }
```

The exit condition. Two things to notice:

**The settle error is measured to the final target** — `profile.distance() -
current` — not to the moving setpoint the PID is chasing. Those are different
questions. The PID asks "am I on plan?"; the settler asks "am I *there* yet?"

**It is gated on `is_done`.** The settler is not even consulted until the
profile has fully played out. Without that gate, a robot that got ahead of its
plan (an overly strong feedforward, or a push from a defending robot) could
land inside the tolerance band early and declare the move finished while the
profile still had commanded motion left to deliver.

`&&` short-circuits, so `settler.update` is not called at all before the
profile finishes — which also means the settle timer cannot accumulate early
credit.

```cpp
    pros::Task::delay_until(&loop_stamp, kLoopMs);
  }
}
```

**`delay_until` versus `delay`.** This is a real distinction worth
understanding.

`pros::delay(10)` sleeps for 10 ms *starting now*. If the loop body took 3 ms,
the period is 13 ms, and it varies with however long the body happened to
take. The loop drifts and the effective frequency is unpredictable — which
matters because `kLoopSeconds` is hard-coded as the PID's `dt`.

`delay_until(&loop_stamp, 10)` sleeps until `loop_stamp + 10` in *absolute*
time, then advances `loop_stamp` by exactly 10. The body's duration is absorbed
into the wait, so the loop runs at a true 100 Hz as long as the body stays
under 10 ms. This is the standard way to write a fixed-rate control loop.

And `&loop_stamp` is the one genuine pointer in the project — address-of, so
the function can advance our variable. See [2.6](#26-references--versus-pointers).

### 5.5 The feedforward physics

The PID is reactive: it needs to see an error before it does anything. But we
already know a lot about what voltage produces what motion, so there is no
reason to make it rediscover that every loop.

A DC motor at steady state obeys:

$$V = k_e \omega + I R$$

where $k_e \omega$ is the back-EMF (the motor generating against itself as it
spins) and $IR$ is the resistive drop. Torque is proportional to current,
$\tau = k_t I$, so:

$$V = k_e \omega + \frac{R}{k_t}\tau$$

The torque has to cover friction plus whatever is left to accelerate the
robot's mass, $\tau = \tau_{\text{friction}} + J\alpha$. Substituting and
regrouping into terms we can measure:

$$\boxed{V = k_S \cdot \operatorname{sign}(v) \;+\; k_V \cdot v \;+\; k_A \cdot a}$$

| Term | Physically | How to find it |
|---|---|---|
| $k_S$ | Voltage to break static friction | Raise it until the robot just barely creeps |
| $k_V$ | Back-EMF: volts per unit speed | $\dfrac{\text{voltage}}{\text{steady-state speed}}$ at that voltage |
| $k_A$ | Volts per unit acceleration (mass/inertia) | Usually small; leave at 0 |

A first estimate of $k_V$ from the robot's free speed. With 3.25" wheels
direct-driven by a 200 rpm cartridge:

$$v_{\text{free}} = \frac{200}{60} \text{ rev/s} \times \pi \times 3.25 \text{ in} \approx 34 \text{ in/s}$$

$$k_V \approx \frac{12000 \text{ mV}}{34 \text{ in/s}} \approx 350 \text{ mV per in/s}$$

which is where the default comes from. For turns, that same wheel speed about
a 11.5" track gives roughly 340 °/s, so $k_V \approx 12000/340 \approx 35$.

**Three consequences worth internalizing:**

**Leave voltage headroom.** At the 24 in/s cap, $k_V$ alone asks for
$350 \times 24 = 8400$ mV, plus 800 for $k_S$ — 9200 of 12000 before the PID
has contributed anything. Only 2800 mV is left for corrections. Set
`max_velocity` near the free speed and the PID has nothing to work with, so it
cannot fix a lag it detects. This is why `max_velocity` is 24 and not 34.

**Guess $k_V$ *low* if you must guess.** Too small and the robot falls behind,
which the PID notices and corrects. Too large and the feedforward is
*actively overdriving* — the PID has to fight its own feedforward, and it shows
up as overshoot. In simulation a 25% overestimate of $k_V$ was the single
largest source of turn overshoot, worse than any PID gain.

**$k_A$ can stay at zero.** It matters most when acceleration is aggressive.
With `max_acceleration` capped well below what the motors could do, the $k_V$
and $k_S$ terms cover nearly everything.

### 5.6 `wrap_180`

```cpp
double wrap_180(double degrees) {
  degrees = std::fmod(degrees + 180.0, 360.0);
  if (degrees < 0.0)
    degrees += 360.0;
  return degrees - 180.0;
}
```

Reduces any angle to the equivalent one in [-180, 180], which is how you get
"shortest way round." A request to turn to a heading 350° away should turn
-10°, not 350°.

The reason this is three lines rather than one is that **`std::fmod` keeps the
sign of the dividend.** `fmod(-10, 360)` is `-10`, not `350`. So the output
range after the first line is (-360, 360), not [0, 360) as you might hope.
Walking through it:

1. `degrees + 180.0` shifts the wrap point so the seam lands where we want it.
2. `fmod(..., 360.0)` reduces the magnitude, landing in (-360, 360).
3. `if (degrees < 0.0) degrees += 360.0` folds the negative half up, giving
   [0, 360).
4. `- 180.0` shifts back, giving [-180, 180).

Spot checks: 190 → 370 → 10 → 10 → **-170** ✓ (170° counterclockwise). And
-190 → -10 → -10 → 350 → **170** ✓.

**Where it is and is not used** is the interesting part:

- `turn_to` wraps, because an absolute heading request should take the short
  way round.
- `drive_distance` wraps the heading-hold error. Drift is tiny so it rarely
  matters, but it costs nothing.
- `turn_by` does **not** wrap `heading() - start_heading`. That is deliberate.
  `get_rotation()` is unbounded, so the raw difference is already the true
  accumulated rotation, and `turn_by(720)` genuinely means two full circles.
  Wrapping there would cap every turn at 180° and silently break large ones.

### 5.7 `drive_distance`

```cpp
void Chassis::drive_distance(double inches, std::uint32_t timeout_ms) {
  reset_position();

  const double start_heading = heading();
  PID heading_pid(heading_hold);
  TrapezoidProfile profile(inches, drive_tuning.constraints);
```

`reset_position()` zeroes the encoders so `position()` measures *this* move
from zero, which is what the profile's coordinates assume. A side effect worth
knowing: `position()` between moves reports only the most recent move's
travel, not cumulative odometry.

`start_heading` snapshots the heading to hold. A separate `heading_pid` runs
alongside the main one — two independent controllers, one for distance and one
for straightness.

```cpp
  follow_profile(
      profile, drive_tuning, [this] { return position(); },
      [this, &heading_pid, start_heading](double output) {
        const double correction = heading_pid.update(
            wrap_180(start_heading - heading()), kLoopSeconds);
        set_voltage(output - correction, output + correction);
      },
      timeout_ms);

  stop();
}
```

**Lambdas, in detail.** A lambda is an anonymous function defined inline. The
syntax is:

```
[capture list](parameters) { body }
```

The **capture list** is what makes lambdas more than function pointers: it
says which surrounding variables the lambda can see, and how.

`[this] { return position(); }` — the measurement lambda. It captures `this`,
the pointer to the current `Chassis` object, which is what lets it call the
member function `position()`. Without capturing `this` there would be no
object to call it on. No parameters, so the `()` is omitted entirely, which is
legal when the list is empty.

`[this, &heading_pid, start_heading](double output) { ... }` — the apply
lambda, with three captures of two different kinds:

| Capture | Mode | Why |
|---|---|---|
| `this` | by value (a pointer copy) | to call `heading()` and `set_voltage()` |
| `&heading_pid` | **by reference** | must be *the* PID, with its accumulating state |
| `start_heading` | by value | an immutable snapshot; copying is correct and clear |

The `&` on `heading_pid` is doing something essential. Captured-by-value
variables are **const inside the lambda** by default, so calling the non-const
`update()` on a copy would not even compile. And semantically we need the real
one: a PID carries internal state between calls, and a copy would be a
separate controller with its own history.

These lambdas capture references to locals of `drive_distance`. That is safe
because `follow_profile` is called **synchronously** and returns before
`drive_distance`'s scope ends. If the lambdas were stored somewhere and called
later, those references would dangle — the usual hazard with reference
captures.

Each lambda has its own unique, compiler-generated type. That is what the
template parameters `Measure` and `Apply` bind to, and why `follow_profile`
gets specialized separately for drives and turns.

**The differential split.** This is the clever bit:

```cpp
set_voltage(output - correction, output + correction);
```

Subtract the correction from one side and add it to the other. Two things fall
out:

$$\frac{(\text{out} - c) + (\text{out} + c)}{2} = \text{out} \qquad\qquad (\text{out}+c) - (\text{out}-c) = 2c$$

The **average is unchanged**, so the forward motion the profile asked for is
preserved exactly. The **difference is $2c$**, which is what steers. Straight-line
progress and heading correction are completely decoupled — the heading PID can
do whatever it likes without stealing from the distance PID.

Checking the sign, because it is easy to get backwards: if the robot drifts
clockwise, `heading()` exceeds `start_heading`, so `start_heading - heading()`
is negative and `correction` is negative. Then the left side gets
`output - correction`, i.e. *more* voltage, and the right gets less, which
rotates the robot counterclockwise — back toward the original heading. Correct.

`stop()` at the end applies the brake mode once the loop exits for any reason.

### 5.8 `turn_by` and `turn_to`

```cpp
void Chassis::turn_by(double degrees, std::uint32_t timeout_ms) {
  const double start_heading = heading();
  TrapezoidProfile profile(degrees, turn_tuning.constraints);

  follow_profile(
      profile, turn_tuning,
      [this, start_heading] { return heading() - start_heading; },
      [this](double output) { set_voltage(output, -output); }, timeout_ms);

  stop();
}
```

Structurally identical to `drive_distance`, with three substitutions:

- The profile is built in **degrees** with the turn constraints, so the exact
  same trapezoid math now plans an angle.
- `measure` returns heading change instead of distance. No `reset_position()`
  is needed, since we subtract `start_heading` instead of zeroing the sensor.
  (There is no "tare" that would be safe here anyway — `turn_to` needs the
  absolute frame preserved.)
- `apply` sends `(output, -output)`: one side forward, the other backward.
  Equal and opposite spins the robot in place. Compare to the drive's
  `(output - c, output + c)`, which has a nonzero average and so translates.

Both lambdas here capture only by value, so there is nothing to outlive.

```cpp
void Chassis::turn_to(double heading_degrees, std::uint32_t timeout_ms) {
  turn_by(wrap_180(heading_degrees - heading()), timeout_ms);
}
```

Absolute heading implemented in terms of relative: compute the difference,
wrap to the shortest direction, delegate. Asking to turn to 10° while sitting
at 350° gives `wrap_180(10 - 350)` = `wrap_180(-340)` = **+20°**, a short
clockwise nudge rather than a 340° spin the long way.

Reusing `turn_by` rather than duplicating the loop means there is exactly one
place turning logic can be wrong.

---

## 6. `main.cpp` — ports, tuning, and the PROS entry points

### 6.1 `main.h`, briefly

```cpp
#include "main.h"
```

Every PROS `.cpp` starts with this. It is the project's single umbrella header:
it defines `PROS_USE_SIMPLE_NAMES` (which is what makes the short macro names
like `DIGITAL_A` and `ANALOG_LEFT_Y` available instead of
`E_CONTROLLER_DIGITAL_A`), includes `api.h` for all of PROS, declares the five
lifecycle functions inside `extern "C"` (see
[2.16](#216-extern-c-in-mainh)), and — after our edit — includes our three
headers in the C++-only section at the bottom:

```cpp
#ifdef __cplusplus
#include "chassis.hpp"
#include "pid.hpp"
#include "profile.hpp"
#endif
```

They go inside the `#ifdef __cplusplus` guard because they are C++ (classes,
namespaces, templates) and this header must stay includable from C.

You will also see `// IWYU pragma: keep` next to the include in our `.cpp`
files. IWYU is "include-what-you-use," a tool that strips includes it thinks
are unused. It cannot tell that `main.h` is what supplies our `fvl` headers
transitively, so the pragma tells it to leave the line alone.

### 6.2 The robot globals

```cpp
namespace {

// TODO: set this to the port the inertial sensor is actually plugged into.
constexpr std::int8_t kImuPort = 7;

pros::MotorGroup left_drive({1, -2, 3});
pros::MotorGroup right_drive({-4, 5, -6});
pros::Imu imu(kImuPort);
```

Inside an anonymous namespace, so these are private to `main.cpp`
([2.4](#24-the-anonymous-namespace)).

`std::int8_t` for the port because that is PROS's port type: a *signed* 8-bit
integer, where the sign carries meaning. Which brings us to the port lists.

`{1, -2, 3}` is a **braced initializer list** of port numbers, and the
negative sign means *reversed*. On a drivetrain the left and right gearboxes
are mirror images, so half the motors must spin the opposite way for the robot
to go straight. Port 2 is reversed on the left; ports 4 and 6 on the right.
Encoding it in the port number means the rest of the code never has to think
about it — `left_drive.move(100)` moves the whole side forward.

These are **your robot's actual wiring** and the numbers here are inherited
from the PROS template. Verify them.

```cpp
const fvl::ChassisConfig kChassisConfig{
    .wheel_diameter = 3.25,
    .gear_ratio = 1.0,
    .track_width = 11.5,
};

fvl::Chassis chassis(left_drive, right_drive, imu, kChassisConfig);
```

Designated initializers ([2.9](#29-default-member-initializers-and-designated-initializers)),
then the chassis itself, binding the three references.

**A real gotcha here: global construction order.** Globals within a single
translation unit are constructed in the order they are *written*. `chassis`
copies `kChassisConfig` by value in its constructor, so `kChassisConfig` must
already exist — and it does, because it is declared above. Reorder those two
lines and you would copy uninitialized memory.

Across *different* `.cpp` files, initialization order is **unspecified** —
there is no way to know which file's globals go first. That is the real reason
all of these live together in one file: it is the only way to guarantee the
ordering. If you ever split them up, you need a different pattern (a function
returning a `static` local, usually).

### 6.3 `configure_chassis`

```cpp
void configure_chassis() {
  chassis.drive_tuning = {
      .pid = {.kP = 1400.0, .kI = 800.0, .kD = 200.0, .integral_zone = 3.0},
      .ff = {.kS = 800.0, .kV = 350.0, .kA = 0.0},
      .constraints = {.max_velocity = 24.0, .max_acceleration = 50.0},
      .settle = {.tolerance = 0.5,
                 .velocity_tolerance = 2.0,
                 .settle_time_ms = 150},
  };
```

Nested designated initializers assigning a whole `MotionTuning` at once. Note
there is no type name on the right-hand side — the compiler infers
`MotionTuning` from what is being assigned to, and then `PIDGains` and
`FeedForward` for the inner braces from the member types.

The units, spelled out, because this is where mixing them up would bite:

| Field | Units | Meaning |
|---|---|---|
| `kP` | mV per inch | 1 inch of error → 1400 mV |
| `kI` | mV per inch-second | sustained error builds voltage |
| `kD` | mV per (inch/s) | velocity error → voltage |
| `integral_zone` | inches | integrate only inside 3 inches |
| `kS` | mV | constant friction offset |
| `kV` | mV per (inch/s) | 24 in/s → 8400 mV |
| `max_velocity` | inch/s | profile speed cap |
| `max_acceleration` | inch/s² | profile ramp steepness |
| `tolerance` | inches | settle window |
| `velocity_tolerance` | inch/s | must also be this slow |

The turn block is the same shape with degrees everywhere. The numbers differ
by a lot — `kP` is 400 rather than 1400 — because a degree and an inch are
different sized things and the gain absorbs the conversion.

```cpp
  chassis.heading_hold = {
      .kP = 200.0, .kI = 0.0, .kD = 0.0, .integral_zone = 0.0};
```

**Proportional only, on purpose.** A D term here would differentiate the
inertial sensor's noise and inject it straight into the motors: ±0.15° of
jitter over a 10 ms loop is ±30 °/s of fake rate, and any meaningful `kD`
turns that into visible twitching. An I term is also a poor fit — heading
drift during a 1-second move is not a sustained error that needs grinding
out, and an integral would just add lag. Plain P is the right tool.

Why this is a function called later, rather than initializing at global scope:
the gains are set *on* an already-constructed `chassis`, and doing it inside
`initialize()` keeps all startup work in one visible place.

### 6.4 The PROS lifecycle

PROS calls five functions. **None of them are in the anonymous namespace** —
they need external linkage so the kernel can find them, which is the whole
point of the `extern "C"` block in `main.h`.

| Function | When it runs |
|---|---|
| `initialize()` | Once at program start. Everything else is blocked until it returns. |
| `competition_initialize()` | After `initialize()`, while connected to field control but not yet enabled. For auton selectors. |
| `autonomous()` | When enabled in autonomous mode. Its own task. |
| `opcontrol()` | When enabled in driver mode. Its own task. |
| `disabled()` | While disabled by field control. |

```cpp
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Calibrating IMU...");

  configure_chassis();
  left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  chassis.calibrate();

  pros::lcd::set_text(0, "Ready.");
}
```

The message before `calibrate()` matters: calibration blocks for about two
seconds, and without a message the robot looks frozen. PROS warns that
`initialize` should stay under a few seconds, since nothing else can start —
calibration is about the most you want in here.

`set_brake_mode_all` sets every motor in the group. **COAST** means the robot
rolls freely when stopped. **HOLD** would actively drive back to the stopped
position, which is usually better for autonomous accuracy — if your routine
drifts between movements, this is the line to change.

```cpp
void autonomous() {
  for (int i = 0; i < 4; i++) {
    chassis.drive_distance(24.0, 3000);
    chassis.turn_by(90.0, 2000);
  }
}
```

A 24-inch square, which is the standard calibration test: four sides and four
90° turns should end where it started, facing the original direction. Any
error accumulates visibly, and *which* error tells you what to fix — ending
short on every side points at `gear_ratio` or `wheel_diameter`; ending rotated
points at the turn tuning.

Both calls **block** until the motion settles or the timeout expires, so this
reads like straight-line code even though a control loop is running inside
each call. The `3000` and `2000` are timeouts in milliseconds.

```cpp
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    pros::lcd::print(0, "pos %.2f in", chassis.position());
    pros::lcd::print(1, "hdg %.2f deg", chassis.heading());
```

`pros::lcd::print` is printf-style: a format string plus varargs, where
`%.2f` means "a float with 2 decimal places." The format specifier has to
match the argument type or you get garbage — `%d` with a double does not
convert, it misreads the bytes.

This readout is the main tuning instrument. Push the robot a known distance by
hand and compare `pos` to a tape measure, and you have verified
`gear_ratio` and `wheel_diameter` without running a single motor.

```cpp
    if (master.get_digital(DIGITAL_A)) {
      autonomous();
      continue;
    }
```

Run the autonomous routine from driver control for testing, without needing a
competition switch.

`DIGITAL_A` is a short macro name, available because `main.h` defines
`PROS_USE_SIMPLE_NAMES`; the full name is `E_CONTROLLER_DIGITAL_A`.

The code comment says "hold A," which overstates it — the button is checked
once, and a single press runs the *entire* square, about 20 seconds, blocking
the whole time. `continue` then jumps to the next iteration of the `while`,
skipping the joystick code below.

```cpp
    int dir = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    left_drive.move(dir - turn);
    right_drive.move(dir + turn);
    pros::delay(20);
  }
}
```

**Arcade drive.** One stick axis for forward/back, another for rotation,
combined by addition — the same differential split as the heading hold in
[5.7](#57-drive_distance), just driven by a human instead of a PID.

`get_analog` returns -127 to 127, which is exactly `move`'s range, so no
conversion is needed. One rough edge: `dir - turn` can reach ±254, which
`move` clamps to ±127. So at full stick deflection in both axes you lose some
turning authority. That is normal for naive arcade drive and most drivers
never notice; scaling or prioritising one axis is the fix if it bothers you.

`pros::delay(20)` yields the CPU so other tasks can run — mandatory in any
`while (true)` loop. 20 ms is 50 Hz, which is plenty for human input. Note
this is plain `delay`, not `delay_until`: exact timing does not matter here the
way it does in the control loop.

---

## 7. Before the first run

Three things in this code are guesses that nobody has checked against the real
robot. All three will produce wrong motion if left alone.

1. **IMU port** — [`main.cpp:6`](src/main.cpp#L6) says port 7. If the sensor
   is elsewhere, every turn fails and `heading()` returns an error value.
2. **Motor ports and reversals** — [`main.cpp:8-9`](src/main.cpp#L8),
   inherited from the PROS template. Wrong reversals make the robot spin when
   asked to drive straight.
3. **`gear_ratio`** — [`main.cpp:17`](src/main.cpp#L17) is 1.0, meaning
   direct drive. If the drivetrain is geared, every distance is wrong by
   exactly that factor. Count the teeth:
   `gear_ratio = motor gear teeth / wheel gear teeth`.

Then verify in this order, since each step depends on the one before:

| Step | How | Pass condition |
|---|---|---|
| 1. Units | Push the robot 24" by hand, read `pos` on the screen | Within an inch |
| 2. Heading | Spin the robot 360° by hand, read `hdg` | Within a few degrees |
| 3. $k_V$ | Drive at a fixed voltage, read steady-state speed | $k_V$ = mV / (in/s) |
| 4. $k_S$ | Raise until the robot just creeps | Barely moves |
| 5. One move | `drive_distance(24)` | Stops within 0.5" |
| 6. One turn | `turn_by(90)` | Stops within 1.5° |
| 7. The square | Hold A in driver control | Ends roughly where it started |

Steps 1 and 2 need no tuning at all and catch the most common mistakes, which
is why they come first.

## 8. Gotchas worth remembering

- **`kI` is not optional.** Static friction means a pure-P controller parks
  about $V_f/k_P$ short of the target and stops. See
  [3.9](#39-why-p-alone-always-stops-short-the-friction-deadband).
- **Conservative gains are not automatically safe.** Too *low* a gain fails to
  settle at all. There is a lower bound as well as an upper one.
- **Guess $k_V$ low, never high.** Overestimating it makes the PID fight its
  own feedforward, which shows up as overshoot.
- **Leave voltage headroom.** If `max_velocity × kV` is near 12000, the PID
  has nothing left to correct with.
- **Never filter a signal you feed back into the loop.** The 125 ms lag on the
  velocity estimate is fine for the settle check and would cause oscillation
  inside the controller.
- **`get_rotation()`, not `get_heading()`.** Unbounded rotation is what makes
  turn arithmetic work across the 0/360 seam.
- **Integer division truncates.** `kLoopMs / 1000` is 0. Write `1000.0`.
- **Gains carry units.** They are not portable between robots, between
  gearings, or between inches and degrees.

## 9. If you want to go further

The obvious next additions, roughly in order of usefulness:

- **Odometry** — track field position $(x, y, \theta)$ continuously instead of
  resetting each move. This is what unlocks "drive to a point."
- **Arc / swing turns** — turn while driving. `track_width` is already in
  `ChassisConfig` waiting for this.
- **Asymmetric profile limits** — a separate deceleration limit. Robots can
  usually brake harder than they accelerate, since weight shifts onto the
  front wheels.
- **Motion chaining** — exit a move early at a nonzero velocity and flow into
  the next one, instead of settling to a stop between every step. Much faster
  autonomous routines, and the profile already has the information needed.

## small game engine Epimetheus
goal is to create an engine that supports main, basic features like:
* Artificial intelligence (perhaps just simple AI utilities, like pathfinding algorithms)
* Saving all or part of the game state (for suspending and restarting at a later time or saving high scores).
* A quite bareboned Network/Lobby implementation
* More abstract handling of multiple controllers (DropIn/DropOut during gaming sessions, like see Resident Evil 5 Coop) - maybe event-based
* GUI (more with ImGui)
* Script Engine (LUA, C#Script, ...)
* Dynamically Refreshed Assets
* Physics
* Particle Engine
* 2D Cameras
* Input Handling
* Screen Management (Menus, Pause Screen, etc.)
* Sprite ( Animation, Sprite Sheets)
* Sound Management

butt according to stackoverflow **all hail stackoverflow**:
```
You're approaching it in an upside-down manner.

What should be in your engine is the following:

All the code that turned out to be common between your first and your second game.

First, write a game. Don't write an engine, because, as you have found out, you don't know what it should contain, or how it should be designed. Write a game instead.

Once you have that game, write another game. Then, when you have done that, examine the second game's code. How much of it was reused from the first game?
```

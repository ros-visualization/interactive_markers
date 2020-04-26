^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package interactive_markers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.12.0 (2020-04-26)
-------------------
* [maintanence] fix signed / unsigned warning
* [maintanence] Migration to tf2
* Contributors: Robert Haschke

1.11.5 (2020-04-05)
-------------------
* [feature]     InteractiveMarkerServer::erase(): return false for unknown marker (`#43 <https://github.com/ros-visualization/interactive_markers/issues/43>`_)
* [fix]         Avoid overriding warning status messages (Fixes `#34 <https://github.com/ros-visualization/interactive_markers/issues/34>`_)
* [fix]         Fixed the problem that `*.hpp` was not installed (`#66 <https://github.com/ros-visualization/interactive_markers/issues/66>`_)
* [fix]         Make the Timer daemon survive /clock jumping back in a playback loop (`#47 <https://github.com/ros-visualization/interactive_markers/issues/47>`_)
* [maintanence] Modernize package.xml and add missing std_msgs in CATKIN_DEPENDS
* [maintanence] Use setuptools instead of distutils (`#67 <https://github.com/ros-visualization/interactive_markers/issues/67>`_)
* [maintanence] Bump cmake version to 3.0.2 (`#64 <https://github.com/ros-visualization/interactive_markers/issues/64>`_)
* [maintanence] Windows compatibility
  * Use C++11 portable sleep (`#63 <https://github.com/ros-visualization/interactive_markers/issues/63>`_)
  * Symbol visibility for shared libraries (`#62 <https://github.com/ros-visualization/interactive_markers/issues/62>`_)
  * Fix install location on Windows (`#39 <https://github.com/ros-visualization/interactive_markers/issues/39>`_)
* Contributors: Alejandro Hernández Cordero, David Gossow, Fabian Maas, Jacob Perron, Robert Haschke, Sean Yen, Shane Loretz, shela

1.11.4 (2018-04-16)
-------------------
* Fixed a crash when updates arrive, or are being processed, while shutdown is called (`#36 <https://github.com/ros-visualization/interactive_markers/issues/36>`_)
* Contributors: Simon Schmeisser

1.11.3 (2016-08-24)
-------------------
* The ``processFeedback`` function of the menu handler no longer catches the ``KeyErrors`` of the feedback_cb.
  See: `#29 <https://github.com/ros-visualization/interactive_markers/issues/29>`_
* Added the ``empty()`` and ``size()`` members to ``InteractiveMarkerServer`` interface.
  See: `#30 <https://github.com/ros-visualization/interactive_markers/issues/30>`_
* Contributors: Blake Anderson, Guglielmo Gemignani

1.11.2 (2016-08-24)
-------------------
* Fix build when disabling tests with ``-DCATKIN_ENABLE_TESTING=OFF``.
  See: `#26 <https://github.com/ros-visualization/interactive_markers/issues/26>`_
* Fix use of uninitialized variables.
  See: `#24 <https://github.com/ros-visualization/interactive_markers/issues/24>`_
* Fix potential segfault when shutting down.
  See: `#25 <https://github.com/ros-visualization/interactive_markers/issues/25>`_
* Contributors: Alexis Ballier, David Gossow, Max Schwarz

1.11.1 (2014-12-16)
-------------------
* Added explicit keyword argument queue_size for publisher in Python code and use the same default queue_size value as C++.
* Fixed a SEGFAULT in setPose reported in `#18 <https://github.com/ros-visualization/interactive_markers/issues/18>`_
  Previously, calling setPose() on an interactive marker causes a SEGFAULT
  if applyChanges() was not called on the server at least once since the
  marker was created. I traced the actual SEGFAULT to the doSetPose
  function. The value of header passed from setPose() is invalid because,
  in this case, marker_context_it = marker_contexts\_.end().
  I added a check for this case and, if there is no marker is present,
  instead use the header from the pending update.
* Contributors: David Gossow, Mike Koval, William Woodall, ipa-fxm

1.11.0 (2014-02-24)
-------------------
* Adding William Woodall as maintainer
* fix threading bugs
  Fix locking of data structures shared across threads.
* Contributors: Acorn Pooley, William Woodall, hersh

1.10.2 (2014-02-03)
-------------------
* fix regression in menu_handler.py
  fixes `#14 <https://github.com/ros-visualization/interactive_markers/issues/14>`_
* Contributors: William Woodall

1.10.1 (2014-01-27)
-------------------
* cleanup python code and package contents
* remove useless dependencies
* Contributors: Vincent Rabaud, William Woodall

1.10.0 (2014-01-23)
-------------------
* remove debug statement that could produce segfault; init_it->msg->markers may be empty
* Contributors: Filip Jares

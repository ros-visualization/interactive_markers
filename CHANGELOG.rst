^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package interactive_markers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

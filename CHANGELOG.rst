^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package interactive_markers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.10 (2014-04-16)
-------------------
* fix threading bugs
  Fix locking of data structures shared across threads.
* Contributors: Acorn Pooley

1.9.9 (2014-01-23)
------------------
* remove debug statement that could produce segfault; init_it->msg->markers may be empty
* Contributors: David Gossow, Filip Jares

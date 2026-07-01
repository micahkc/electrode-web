# Topic Conventions

Topics use the form:

```text
vehicle/{id}/{domain}/{name}
```

Initial topics:

```text
vehicle/{id}/state/pose
vehicle/{id}/state/velocity
vehicle/{id}/state/attitude
vehicle/{id}/state/battery
vehicle/{id}/state/link
vehicle/{id}/state/mode
vehicle/{id}/state/localization
vehicle/{id}/event
vehicle/{id}/cmd/arm
vehicle/{id}/cmd/disarm
vehicle/{id}/cmd/mode
vehicle/{id}/cmd/land
vehicle/{id}/cmd/return
vehicle/{id}/mission/upload
vehicle/{id}/mission/status
vehicle/{id}/param/get
vehicle/{id}/param/set
vehicle/{id}/log
```

Every registered topic defines schema name, expected rate, stale timeout, display units, logging behavior, and command authority requirement.


# Introduction
This is a simple notification system. It is an Autoload or Global variable so it should be accessible in all scenes.

# How to use
Notifications can be created using the following command in gdscript
```
NotificationCanvasGlobal.add_notification("EXAMPLE NOTIFICATION", 3.0)
```

> NOTE: By default notifications have a lifetime of -1


This will create a notification that says "EXAMPLE NOTIFICATION" and will exist for 3 seconds.

Using a negative value for the notification (ex: -1) will let the notification last forever

Notifications can be closed by clicking on them or by clicking on clear all.

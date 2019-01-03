#!/bin/bash
rosservice call --wait /odom/reset &
exec "$@"

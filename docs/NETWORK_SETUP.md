# Networking Setup Guide

This guide contains all the info necessary for developing on the onboard computer in various scenarios. This will be updated over time as the setup gets more streamlined.

## Networking Must-Haves

below are some must-haves for our stack to allow for proper development/testing of the car:

- [x] ssh connection to car through ethernet LAN
- [x] ability to connect to the car via ssh from anywhere, even off campus
- [ ] ability to create a wireless local area network where we can ssh into the car without needing to be wired to it

## Remote Development From Anywhere

As of right now, remotely ssh-ing into the jetson is possible through a [reverse proxy](https://en.wikipedia.org/wiki/Reverse_proxy) being hosted on a [digital ocean cloud droplet](https://docs.digitalocean.com/products/droplets/). We have [rathole](https://github.com/rapiz1/rathole/tree/main) acting as the reverse proxy, forwarding ssh requests from your laptop to the jetson. For this to work, both the jetson and your laptop have to be connected to the internet. The droplet setup was done following [this tutorial](https://noway.moe/unix/reverse-proxy/).

In order to get ssh access to the jetson, you must ask a lead for permission and help. The process requires adding your public ssh key to the authorized hosts on the digital ocean droplet and on the jetson. Once done, you can ssh into the jetson via the command `ssh -p 8022 nvidia@137.184.234.180`.

**Note:** expect a lot of latency when developing remotely, as the digital ocean server is currently in san francisco (was the cheapest option).

## Remote Development From Clubs Room

We have a small ethernet switch set up for connecting to jetson through ethernet! This is to be used when low-latency development is required while working in the clubs room. It also works for testing slow movements or autonomy elements not requiring movements (as we don't really want to be zooming around while hardwired into ethernet).

Plug in an ethernet cable from the switch to the jetson, and one from the switch into your laptop, and then follow the setup guide for your operating system below. Once set up, try pinging the jetson using the command `ping 192.168.1.100` in a terminal. If that works, you can ssh into the jetson using `ssh -X nvidia@192.168.1.100` (the -X flag allows X11-forwarding for GUI's, though I haven't gotten this working in the docker containers yet).

**Note**: this process also requires you to ask a lead for permission and help, as your public ssh key needs to be authorized on the jetson like in the remote connection setup.

#### Linux Setup

All that is required is just to manually set ipv4 in network manager settings:

- **IP** is 192.168.1.X (ask a lead for what X value you can use between 1-255)
- **subnet mask** is 255.255.255.0 (i.e we only get last 8 bits for host range, other 24 bits are for network ip)
- **gateway** is 192.168.1.1
- **DNS** is 8.8.8.8 and 8.8.4.4

#### Mac Setup

TODO

#### Windows Setup

TODO

## Remote Connection For Testing

as of right now, remote ssh through reverse proxy will have to do. There are plans to buy a wireless Access Point to allow for a low-latency LAN connection over wifi (so we don't have to worry about ripping out ethernet cables all the time).

- If a data center in Alberta is pursued for remote ssh that ends up having low enough latency, we could just stick with that and not pursue the wireless Access Point. A wireless Access Point will likely eventually be required for competition though.

If/When the wiress Access Point gets set up, the connection process will likely look similar to the ethernet connection process.

## To-Do

There is still lots of room for improvement in this setup. Here are some potential updates:

- change over to [cloudflare](https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/use-cases/ssh/) or another cloud server provider that has data centers in alberta. We don't need anything crazy, literally just need to be able to ssh into the jetson
  - another option is to set up a local server on campus and get it a static IP address. This lets us do a bunch of other things with very low latency as well
- there is a bug (at least I think) where the jetson can't handle remote ssh connections while it is connected through ethernet to a LAN. Someone needs to look further into this.
- we run a very basic netplan setup on the jetson (literally just handing control to network manager). If more complex network setup's need to be created, someone will have to look further into what can be done with netplan.
- a proper zerotier setup could be put in place as a backup plan for remote connections in case the data center option ceases to work (I highly doubt this ever happens tho)

## Notes

below are some links and notes for general tips w.r.t networking

- [X11 Forwarding](https://goteleport.com/blog/x11-forwarding/) tutorial for trying to get GUI forwarding from container to local laptop over ssh connection
- if you ever need to see the file that gets created for the NetworkManager settings you just set (either locally on linux or on the jetson), go to `/etc/NetworkManager/system-connections` and find the name of your wired connection that you configured, and `cat` it as root user
  - generally speaking, most of the networking config files on the jetson will reside in a subdirectory of the `etc/` directory.

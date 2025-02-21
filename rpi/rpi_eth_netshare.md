
## Inside Ubuntu PC
 <div>
 <img src="rpi_eth_netshare.png" alt="Ubuntu Settings"> 
 </div>
```
  sudo ifconfig ethxx 192.168.129.10
```
<div> <i>Assuming wlp1s0 ip is 192.168.129.250, </i></div>

```
  sudo sysctl -w net.ipv4.ip_forward=1
```
```
 sudo vi /etc/sysctl.conf
```
<div> <i> Uncomment net.ipv4.ip_forward=1, </i></div>

```
  sudo iptables -t nat -A POSTROUTING -o wlp1s0 -j MASQUERADE
  sudo apt install iptables-persistent
  sudo sh -c "iptables-save > /etc/iptables/rules.v4"  
  sudo systemctl restart NetworkManager
  ip route show
```
Expected output --> 
<div><i> textdefault via 192.168.129.111 dev wlp1s0 proto dhcp src 192.168.129.250 metric 600 </i></div>
## Inside Raspberry pi

```
ifconfig eth0 192.168.129.12
sudo ip route add default via 192.168.129.10 dev eth0
```


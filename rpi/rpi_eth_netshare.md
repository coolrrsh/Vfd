
<h2>Inside Ubuntu </h2> 
 
```
  sudo ifconfig ethxx 192.168.129.10
```
<div> Assuming wlp1s0 ip is 192.168.129.250, </div>

```
  sudo sysctl -w net.ipv4.ip_forward=1
```
```
 sudo vi /etc/sysctl.conf
```
<div>  Uncomment net.ipv4.ip_forward=1, </div>

```
  sudo iptables -t nat -A POSTROUTING -o wlp1s0 -j MASQUERADE
  sudo apt install iptables-persistent
  sudo sh -c "iptables-save > /etc/iptables/rules.v4"  
  sudo systemctl restart NetworkManager
  ip route show
```
Expected output --> 
<div style="color: red;">
default via 192.168.156.31 dev wlp1s0 proto dhcp src 192.168.156.250 metric 600 <br>
172.17.0.0/16 dev docker0 proto kernel scope link src 172.17.0.1 linkdown <br>
192.168.118.0/24 dev enx806d97057f55 proto kernel scope link src 192.168.118.10 metric 100 <br>
192.168.156.0/24 dev wlp1s0 proto kernel scope link src 192.168.156.250 metric 600 <br>
</div>

<h2>Inside Raspberry pi </h2>

```
ifconfig eth0 192.168.129.12
sudo ip route add default via 192.168.129.10 dev eth0
```


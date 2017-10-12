Vagrant.configure("2") do |config|
  config.vm.box = "ubuntu/xenial64"
  config.ssh.forward_agent = true
  config.vm.provision :shell, path: "vagrant/root_bootstrap.sh"
  config.vm.provision :shell, path: "vagrant/user_bootstrap.sh", privileged: false
  config.vm.provision :shell, path: "vagrant/root_finish_bootstrap.sh"
end

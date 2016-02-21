if (process.argv.length < 4) {
    console.log("USAGE: node run_route.js 0x1234.... '[[65.32, 49.12, 15], ...'");
    process.exit(1);
}

var address = process.argv[2];
var points  = JSON.parse(process.argv[3]);
var Web3    = require('web3'); 
var web3 = new Web3();
web3.setProvider(new web3.providers.HttpProvider('http://localhost:8545'));
var drone_employee_abi = [{"constant":false,"inputs":[{"name":"_response","type":"address"}],"name":"setRoute","outputs":[],"type":"function"},{"constant":false,"inputs":[{"name":"_route_id","type":"uint32"}],"name":"flightDone","outputs":[],"type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"subscribers","outputs":[{"name":"","type":"address"}],"type":"function"},{"constant":false,"inputs":[{"name":"latitude","type":"int256"},{"name":"longitude","type":"int256"},{"name":"altitude","type":"int256"}],"name":"addCheckpoint","outputs":[],"type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"checkpoints","outputs":[{"name":"","type":"address"}],"type":"function"},{"constant":false,"inputs":[],"name":"initROS","outputs":[],"type":"function"},{"constant":false,"inputs":[],"name":"takeFlight","outputs":[],"type":"function"},{"constant":false,"inputs":[{"name":"_name","type":"string"},{"name":"_type","type":"string"}],"name":"mkPublisher","outputs":[{"name":"","type":"address"}],"type":"function"},{"constant":false,"inputs":[{"name":"_name","type":"string"},{"name":"_type","type":"string"},{"name":"_callback","type":"address"}],"name":"mkSubscriber","outputs":[{"name":"","type":"address"}],"type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"publishers","outputs":[{"name":"","type":"address"}],"type":"function"},{"inputs":[{"name":"_controller","type":"address"}],"type":"constructor"}];

var contract = web3.eth.contract(drone_employee_abi).at(address);
points.forEach(function(p, i, arr) {
    contract.addCheckpoint(
        p[0] * 1000000, p[1] * 1000000, p[2] * 1000000,
        {from: web3.eth.accounts[0], gas: 3000000});
});
contract.takeFlight({from: web3.eth.accounts[0], gas: 3000000});

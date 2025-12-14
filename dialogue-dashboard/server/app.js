var express = require('express');
var bodyParser = require('body-parser');
var app = express();
var DATA_NUMBER = 30;
// フィールド名とモジュールのつながり解決用
function getModName(field) {
    switch (field) {
        case 'is_final':
            return 'asr';
        case 'reply':
            return 'nlg';
        case 'prevgrad':
        case 'frequency':
        case 'grad':
        case 'power':
        case 'zerocross':
            return 'sa';
    }
}
var chatData = { turn: 0, speaker: "", bot: "" };
var db = [];
var modStatus = {
    sm: false,
    lu: false,
    dm: false,
    rc: false,
    nlg: false,
    ss: false,
    dr: false
};
var startTime = Date.now();
var count = 0;
// urlencodedとjsonは別々に初期化する
app.use(bodyParser.urlencoded({
    extended: true
}));
app.use(bodyParser.json());
app.listen(3000);
console.log('Server is online. Port 3000.');
app.post('/data', function (req, res) {
    if (!db.length)
        startTime = Date.now();
    var data = req.body;
    console.log(data);
    var turn = JSON.parse(data.turn);
    var asr = JSON.parse(data.asr);
    var nlg = JSON.parse(data.nlg);
    var sa = JSON.parse(data.sa);
    var now = Date.now();
    var time = now - startTime;
    db.push({ turn: turn, asr: asr, nlg: nlg, sa: sa, time: time });
    chatData.turn = turn;
    chatData.speaker = asr.you;
    chatData.bot = nlg.reply;
    res.send('POST request to the homepage');
});
app.get('/getGraphData', function (req, res) {
    if (db.length < DATA_NUMBER)
        res.send(undefined);
    var frequency_value = setGraphData("sa", "frequency");
    var power_value = setGraphData("sa", "power");
    var zerocross_value = setGraphData("sa", "zerocross");
    var frequency = { name: "frequency", label: frequency_value, value: frequency_value };
    var power = { name: "power", label: power_value, value: power_value };
    var zerocross = { name: "zerocross", label: zerocross_value, value: zerocross_value };
    var dataset = { data: [frequency, power, zerocross] };
    res.send(dataset);
});
app.get('/getLog', function (req, res, next) {
    console.log('getLog');
    count++;
    var rejson = { data: chatData, count: count };
    res.send(rejson);
});
app.get('/getCsvData', function (req, res) {
    var name = req.query.name;
    console.log(name);
    var sendData = setCSVdata(name);
    res.send({ data: sendData });
});
app.post('/modstatus', function (req, res) {
    var data = req.body;
    modStatus = JSON.parse(data.data);
    console.log(modStatus);
    res.send('ok');
});
app.get('/getModStatus', function (req, res) {
    res.send({ data: modStatus });
});
function setGraphData(mod, field) {
    try {
        var target = db.slice(-1 * DATA_NUMBER);
        var result = [];
        for (var _i = 0, target_1 = target; _i < target_1.length; _i++) {
            var rosData = target_1[_i];
            result.push(rosData[mod][field]);
        }
        return result;
    }
    catch (err) {
        console.log("ERR", err);
        return [];
    }
}
function setCSVdata(name) {
    try {
        var mod_1 = getModName(name);
        var sendData_1 = [];
        db.map(function (rosData) {
            sendData_1.push({ target: rosData[mod_1][name], time: rosData["time"] });
        });
        return sendData_1;
    }
    catch (err) {
        console.log("ERR", err);
        return [];
    }
}

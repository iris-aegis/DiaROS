const express = require('express');
const bodyParser = require('body-parser');

const app = express();

const DATA_NUMBER = 30;

interface ChatLog {
    turn: number
    speaker: string
    bot: string
}
interface IRosData {
    turn: number
    asr:  { you: string, is_final: boolean }
    nlg:  { reply: string }
    sa:   { prevgrad:  number, 
            frequency: number, 
            grad:      number, 
            power:     number, 
            zerocross: number }
    time: number
}

// フィールド名とモジュールのつながり解決用
function getModName( field: string ) {
    switch( field ) {
        case 'is_final' : 
            return 'asr';
        case 'reply'    : 
            return 'nlg';
        case 'prevgrad' :
        case 'frequency':
        case 'grad'     :
        case 'power'    :
        case 'zerocross':
            return 'sa';
    }
}

const chatData: ChatLog = { turn: 0, speaker: "", bot: "" };
const db: IRosData[] = [];
let modStatus = { 
                    sm:  false,
                    lu:  false,
                    dm:  false,
                    rc:  false,
                    nlg: false,
                    ss:  false,
                    dr:  false
                };
let startTime: number = Date.now();

let count: number = 0;

// urlencodedとjsonは別々に初期化する
app.use(bodyParser.urlencoded({
    extended: true
}));
app.use(bodyParser.json());

app.listen(3000);
console.log('Server is online. Port 3000.');

app.post('/data', (req:any, res:any) => {
    if(!db.length) startTime = Date.now();
    const data = req.body;
    console.log(data);
    const turn = JSON.parse(data.turn);
    const asr =  JSON.parse(data.asr);
    const nlg =  JSON.parse(data.nlg);
    const sa =   JSON.parse(data.sa);
    const now = Date.now();
    const time = now - startTime;
    db.push( { turn: turn, asr: asr, nlg: nlg, sa: sa, time: time } );

    chatData.turn = turn;
    chatData.speaker = asr.you;
    chatData.bot = nlg.reply;
    
    res.send('POST request to the homepage');

})

app.get('/getGraphData', (req:any, res:any) => {
    if( db.length < DATA_NUMBER ) res.send(undefined); 
    const frequency_value = setGraphData("sa", "frequency");
    const power_value = setGraphData("sa", "power");
    const zerocross_value = setGraphData("sa", "zerocross");

    const frequency = { name: "frequency", label: frequency_value, value: frequency_value };
    const power = { name: "power", label: power_value, value: power_value };
    const zerocross = { name: "zerocross", label: zerocross_value, value: zerocross_value };

    const dataset = { data: [ frequency, power, zerocross ] };
    res.send(dataset);
})

app.get('/getLog', (req:any, res:any, next:any) => {
    console.log('getLog');
    count++;
    const rejson = { data: chatData, count: count };
    res.send(rejson);
})

app.get('/getCsvData', (req:any, res:any) => {
    const name = req.query.name;
    console.log(name);
    const sendData = setCSVdata(name);
    res.send({data: sendData});
}) 

app.post('/modstatus', (req:any, res:any) => {
    const data = req.body;
    modStatus = JSON.parse(data.data);
    console.log(modStatus);
    res.send('ok');
})

app.get('/getModStatus', (req:any, res:any) => {
    res.send({data: modStatus});
})

function setGraphData( mod:string, field:string ) {
    try {
        const target: IRosData[] = db.slice(-1 * DATA_NUMBER);
        let result = [];
        for( const rosData of target ) {
            result.push(rosData[mod][field])
        }
        return result;
    } catch (err) {
        console.log("ERR", err);
        return [];
    }
}

function setCSVdata( name: string ) {
    try {
        const mod = getModName(name);
        const sendData: { target: number, time: number  }[] = [];
        db.map((rosData:IRosData) => {
            sendData.push( { target: rosData[mod][name], time: rosData["time"] } );
        });
        return sendData;
    } catch (err) {
        console.log("ERR", err);
        return [];
    }
}
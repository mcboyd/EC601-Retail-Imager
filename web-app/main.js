var express = require('express');
var app = express();
var http=require('http').Server(app);
var io=require('socket.io')(http);
var mysql=require('mysql');
const execFile = require('child_process').execFile;
var sqlResult = " ";
var jobStatus = "Stopped";
var connection = mysql.createConnection({
    host     : 'localhost',
    user     : 'root',
    password : 'retail-Imager1',
    port: '3306',
    database: 'retail_imager',
});

connection.connect();

app.use(express.static('public'));

app.get('/start',function(req,res) {
    if (jobStatus == "Stopped") {
        jobStatus = "Started";
        startImaging();  // Runs every 90 seconds
    } else {
        jobStatus = "Stopped";
    }
    console.log(jobStatus);
    io.emit('jobStatus', jobStatus);
    res.sendStatus(200);
});

app.get('/',function(req,res) {
    res.sendFile( __dirname + "/views/start.html" );
  });

app.get('/process_get',function(req,res) {
    response = {
        "prodid":req.query.prodid,
    };
    sql = 'SELECT * FROM product where prodid = '+response.prodid;
    //start searching
    connection.query(sql,function (err, result) {
        if(err){
            //[SELECT ERROR] -  connect ECONNREFUSED 127.0.0.1:3306
            console.log('[SELECT ERROR] - ',err.message);
            return;
        }
        console.log('--------------------------SELECT----------------------------');
        console.log(result);
        sqlResult=result[0];
        sqlResult = JSON.stringify(sqlResult);
        console.log('------------------------------------------------------------\n\n');
        io.emit('match', sqlResult);
    });
    res.sendStatus(200);
});

io.sockets.on('connection',function(socket) {
    //console.log('good');
    //socket.broadcast.emit('users',Result);
    if (sqlResult != " ") {
        io.emit('match', sqlResult);
    }
    io.emit('jobStatus', jobStatus);
});


var startImaging = function() {
    if (jobStatus != "Stopped") {
        child = execFile('notepad.exe', (error, stdout, stderr) => {
            if (error) {
                console.error('stderr', stderr);
                throw error;
            }
            console.log('stdout ', stdout);
        });
        setTimeout(startImaging, 5000);
    }
}

 
http.listen(8081,function(){
    console.log('listen: 8081');
  });

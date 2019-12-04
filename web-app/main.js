var express = require('express');
var app = express();
var http=require('http').Server(app);
var io=require('socket.io')(http);
var mysql=require('mysql');
var Result = " ";
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
    console.log("start recieve");
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
        Result=result[0];
        Result = JSON.stringify(Result);
        console.log('------------------------------------------------------------\n\n');
        io.emit('match', Result);
    });
    res.sendStatus(200);
});

io.sockets.on('connection',function(socket) {
    //console.log('good');
    //socket.broadcast.emit('users',Result);
});



 
http.listen(3000,function(){
    console.log('listen: 3000');
  });


var app=require('express')();
var http=require('http').Server(app);
var io=require('socket.io')(http);
var mysql=require('mysql');
var Result = " ";
var connection = mysql.createConnection({
    host     : 'localhost',
    user     : 'root',
    password : '123456',
    port: '3306',
    database: 'retail_imager',
});


connection.connect();

app.get('/start',function(req,res) {
    console.log("start recieve");
    res.status(200).send({ success: true });
});

app.get('/',function(req,res) {
    res.sendFile(__dirname + '/FRONT-END.html');
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
        Result=result;
        Result = JSON.stringify(Result);
        can = Result.replace(/:/g,'');
        can = (can || "").split('"');
        console.log('------------------------------------------------------------\n\n');
    });
    res.sendFile(__dirname + '/FRONT-END.html');    
});

io.sockets.on('connection',function(socket) {
    console.log('good');
    socket.broadcast.emit('users',Result);
});



 
http.listen(3000,function(){
    console.log('listen: 3000');
  });
  
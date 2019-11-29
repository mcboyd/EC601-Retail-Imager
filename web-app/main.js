var express = require('express');
var mysql  = require('mysql');
var app = express();
const exphbs = require('express-handlebars');
var server = require('http').Server(express);

var response={};

var io = require('socket.io')(server);
var Result='';
var can = [];

app.use(express.static(__dirname + '/public'));

app.engine('html', exphbs({
    layoutsDir: 'views',
    defaultLayout: 'layout',
    extname: '.html'
}));
app.set('view engine', 'html');

//send the html file
app.get('/', function (req, res) {
    res.sendFile( __dirname + "/views/start.html" );
})


//after getting the id, connect the mysql db
app.get('/process_get', function (req, res) {
    // JSON file output
    response = {
        "prodid":req.query.prodid,
    };
    //create the connection
    var connection = mysql.createConnection({
        host     : 'localhost',
        user     : 'root',
        password : '123456',
        port: '3306',
        database: 'retail_imager',
    });
    //connect thr DB
    connection.connect();
    //search the db
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
    //end the connection
   connection.end();
   res.render('pre', {
    layout: false,
    title: "Imager",
    personInfoList: [{
        id: (can[2] || "").split(",")[0],
        name: can[13],
        brand:can[9],
        sku: can[5],
        price: (can[20] || "").split(",")[0],
        quan: can[17],
        num: "product" + (can[2] || "").split(",")[0] + ".jpg"// need to redefine the name of the image according to the local address
    }]
});
})




server = app.listen(8081, function () {
     var host = server.address().address
     var port = server.address().port
     console.log("listening on http://%s:%s", host, port)
 })
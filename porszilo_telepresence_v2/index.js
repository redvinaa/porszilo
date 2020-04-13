#! /usr/bin/nodejs

const express = require('express');
const path = require('path');

const app = express();

app.use(express.json());
app.use(express.static(path.join(__dirname, 'static')));

app.set('view engine', 'pug');
app.set('views', path.join(__dirname, 'views'));


app.get('/', (req, res)=>{
	res.render('telepresence');
});

app.listen(5000, ()=>console.log("Server ready!"));

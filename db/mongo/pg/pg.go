package pg

import (
	"database/sql"
	"fmt"
	"log"
	"os"
)

func ConnectPG() *sql.DB {
	usr, _ := os.LookupEnv("PG_USER")
	pass, _ := os.LookupEnv("PG_PASSWORD")
	dbName, _ := os.LookupEnv("PG_DATABASE")
	host, _ := os.LookupEnv("PG_HOST")
	port, _ := os.LookupEnv("PG_PORT")

	connStr := fmt.Sprintf("postgres://%s:%s@%s:%s/%s?sslmode=disable", usr, pass, host, port, dbName)
	db, err := sql.Open("postgres", connStr)

	if err != nil {
		log.Fatalln(err)
	}

	err = db.Ping()

	if err != nil {
		log.Fatalln(err.Error())
	}

	fmt.Println("pg connected!")

	return db
}

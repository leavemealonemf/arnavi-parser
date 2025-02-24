package mongo

import (
	"context"
	"fmt"
	"log"

	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
)

func Connect(ctx context.Context, connStr string) (*mongo.Client, error) {
	clientOptions := options.Client().ApplyURI(connStr)
	client, err := mongo.Connect(ctx, clientOptions)
	if err != nil {
		defer func() {
			e := client.Disconnect(ctx)
			if e != nil {
				fmt.Println("Client disconnect err")
			}
		}()
		return nil, fmt.Errorf("Connect to MongoDB error with credentials:\n%v\n", connStr)
	}

	return client, nil
}

func GetAll(ctx context.Context, col *mongo.Collection) {
	curr, err := col.Find(ctx, nil)
	if err != nil {
		log.Fatalln(err.Error())
	}
	var results []any
	if err = curr.All(context.TODO(), &results); err != nil {
		panic(err)
	}
	fmt.Println(results...)
}

func Insert(ctx context.Context, col *mongo.Collection, data interface{}) {
	_, err := col.InsertOne(ctx, data)
	if err != nil {
		fmt.Println("[MONGO] Insert entity err:", err.Error())
	}
}

func FindOneWithOpts(ctx context.Context, col *mongo.Collection, f interface{}, opts *options.FindOneOptions) *mongo.SingleResult {
	return col.FindOne(ctx, f, opts)
}

func FindAll(ctx context.Context, col *mongo.Collection) *mongo.Cursor {
	f := bson.D{}
	curr, err := col.Find(ctx, f)

	if err != nil {
		fmt.Println("[MONGO] Find all err:", err.Error())
		return nil
	}

	return curr
}

func FindAllWithOpts(ctx context.Context, col *mongo.Collection, f interface{}, opts *options.FindOptions) *mongo.Cursor {
	curr, err := col.Find(ctx, f, opts)
	if err != nil {
		fmt.Println("[MONGO] Find all err:", err.Error())
		return nil
	}
	return curr
}

func UpdOne(ctx context.Context, col *mongo.Collection, f interface{}, upd interface{}) *mongo.UpdateResult {
	result, err := col.UpdateOne(ctx, f, upd)
	if err != nil {
		fmt.Println("[MONGO] Update err:", err.Error())
		return nil
	}

	return result
}

func UpdOneScooter(ctx context.Context, col *mongo.Collection, f interface{}, upd interface{}, opts *options.FindOneAndUpdateOptions) *mongo.SingleResult {
	result := col.FindOneAndUpdate(ctx, f, upd, opts)
	if result.Err() != nil {
		log.Printf("Failed to update scooter: %v", result.Err())
	} else {
		log.Println("Successfully updated the latest scooter by timestamp")
	}

	return result
}

/*
 * ServerException.h
 *
 *  Created on: 18.03.2012
 *      Author: leon
 */

#ifndef SERVEREXCEPTION_H_
#define SERVEREXCEPTION_H_

#include <exception>
#include <string>

namespace ros4rsb {

class ServerException: public std::exception {
private:
	std::string error;
public:
	ServerException(std::string error) :
			std::exception(), error(error) {
	}
	virtual ~ServerException() throw () {
	}
	virtual const char* what() const throw () {
		return error.c_str();
	}
};

}

#endif /* SERVEREXCEPTION_H_ */

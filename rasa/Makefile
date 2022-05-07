# ----------------------------------------------------------------------
#  Rasa Commands
# ----------------------------------------------------------------------
SHELL := /bin/bash

#: Create virtualenv 
setup:
	( \
			virtualenv -p python3 venv \
	)

#: Install all in virtualenv
install:
	( \
			source venv/bin/activate; \
			pip3 install -r requirements.txt; \
	)
#: Train and Create model
rasa.train:
	( \
			source venv/bin/activate; \
			rasa train; \
	)
#: Start action server.
rasa.develop.run:
	( \
			source venv/bin/activate; \
			rasa run actions & rasa shell; \
	)

#: Start action server.
rasa.run:
	( \
			source venv/bin/activate; \
			rasa run actions & rasa run --enable-api; \
	)

rasa.debug:
	( \
			source venv/bin/activate; \
			rasa run actions --debug  & rasa run --debug --enable-api; \
	)
	
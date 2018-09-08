package org.visp.engine;

/**
* Class provides Java interface to VISP Engine Service. Is synchronious with native VISPEngine class.
*/
interface VISPEngineInterface
{
    /**
    * @return Return service version
    */
    int getEngineVersion();

    /**
    * Find installed VISP library
    * @param VISP version
    * @return Returns path to VISP native libs or empty string if VISP was not found
    */
    String getLibPathByVersion(String version);

    /**
    * Try to install defined version of VISP from Google Play (Android Market).
    * @param VISP version
    * @return Returns true if installation was successful or VISP package has been already installed
    */
    boolean installVersion(String version);

    /**
    * Return list of libraries in loading order separated by ";" symbol
    * @param VISP version
    * @return Returns VISP libraries names separated by symbol ";" in loading order
    */
    String getLibraryList(String version);
}

package org.visp.engine;

/**
* Class provides Java interface to ViSP Engine Service. Is synchronious with native ViSPEngine class.
*/
interface ViSPEngineInterface
{
    /**
    * @return Return service version
    */
    int getEngineVersion();

    /**
    * Find installed ViSP library
    * @param ViSP version
    * @return Returns path to ViSP native libs or empty string if ViSP was not found
    */
    String getLibPathByVersion(String version);

    /**
    * Try to install defined version of ViSP from Google Play (Android Market).
    * @param ViSP version
    * @return Returns true if installation was successful or ViSP package has been already installed
    */
    boolean installVersion(String version);

    /**
    * Return list of libraries in loading order separated by ";" symbol
    * @param ViSP version
    * @return Returns ViSP libraries names separated by symbol ";" in loading order
    */
    String getLibraryList(String version);
}

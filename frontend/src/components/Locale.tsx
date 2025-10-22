import i18n from "i18next";
import { initReactI18next } from "react-i18next";

import LanguageDetector from "i18next-browser-languagedetector";
import LocaleHelper from "../misc/LocaleHelper";

i18n.use(LanguageDetector).use(initReactI18next).init(LocaleHelper.getConfig());

export default i18n;

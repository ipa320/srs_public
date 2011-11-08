-- MySQL dump 10.13  Distrib 5.1.54, for debian-linux-gnu (i686)
--
-- Host: localhost    Database: srs_gdatabase
-- ------------------------------------------------------
-- Server version	5.1.54-1ubuntu4

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `action`
--

DROP TABLE IF EXISTS `action`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `action` (
  `action_id` int(11) NOT NULL AUTO_INCREMENT,
  `action_type` varchar(30) NOT NULL,
  `action_name` varchar(30) DEFAULT NULL,
  `parameter_list` int(11) DEFAULT NULL,
  PRIMARY KEY (`action_id`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `action`
--

LOCK TABLES `action` WRITE;
/*!40000 ALTER TABLE `action` DISABLE KEYS */;
INSERT INTO `action` VALUES (1,'Observe action','Environment-update',3),(2,'physical action','grasp',5),(3,'physical action','move',1),(4,'Observe action','detect',3);
/*!40000 ALTER TABLE `action` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `component`
--

DROP TABLE IF EXISTS `component`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `component` (
  `component_id` int(11) NOT NULL AUTO_INCREMENT,
  `component_name` varchar(40) DEFAULT NULL,
  PRIMARY KEY (`component_id`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `component`
--

LOCK TABLES `component` WRITE;
/*!40000 ALTER TABLE `component` DISABLE KEYS */;
INSERT INTO `component` VALUES (1,'base'),(2,'sdh'),(3,'cameras'),(4,'arm'),(5,'sdh and arm');
/*!40000 ALTER TABLE `component` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `map`
--

DROP TABLE IF EXISTS `map`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `map` (
  `map_id` int(11) NOT NULL AUTO_INCREMENT,
  `map_name` varchar(45) DEFAULT NULL,
  `map_time` datetime DEFAULT NULL,
  `map_location` point DEFAULT NULL,
  PRIMARY KEY (`map_id`)
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `map`
--

LOCK TABLES `map` WRITE;
/*!40000 ALTER TABLE `map` DISABLE KEYS */;
INSERT INTO `map` VALUES (1,'kitchen',NULL,NULL),(2,'room',NULL,NULL);
/*!40000 ALTER TABLE `map` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `oacs`
--

DROP TABLE IF EXISTS `oacs`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `oacs` (
  `description_id` int(11) NOT NULL AUTO_INCREMENT,
  `symbolic_id` int(11) DEFAULT NULL,
  `object_id` int(11) DEFAULT NULL,
  `action_id` int(11) NOT NULL,
  `component_id` int(11) DEFAULT NULL,
  `Grounded_concept` text,
  PRIMARY KEY (`description_id`),
  KEY `fk_oacs_1` (`action_id`),
  KEY `fk_oacs_2` (`component_id`),
  KEY `fk_oacs_3` (`object_id`),
  KEY `fk_oacs_4` (`symbolic_id`),
  CONSTRAINT `fk_oacs_1` FOREIGN KEY (`action_id`) REFERENCES `action` (`action_id`) ON DELETE NO ACTION ON UPDATE NO ACTION,
  CONSTRAINT `fk_oacs_2` FOREIGN KEY (`component_id`) REFERENCES `component` (`component_id`) ON DELETE NO ACTION ON UPDATE NO ACTION,
  CONSTRAINT `fk_oacs_3` FOREIGN KEY (`object_id`) REFERENCES `object` (`object_id`) ON DELETE NO ACTION ON UPDATE NO ACTION,
  CONSTRAINT `fk_oacs_4` FOREIGN KEY (`symbolic_id`) REFERENCES `symbolic` (`symbolic_id`) ON DELETE NO ACTION ON UPDATE NO ACTION
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `oacs`
--

LOCK TABLES `oacs` WRITE;
/*!40000 ALTER TABLE `oacs` DISABLE KEYS */;
INSERT INTO `oacs` VALUES (1,1,1,3,1,NULL),(2,1,2,3,1,NULL),(3,1,3,3,1,NULL),(4,4,4,2,5,NULL),(5,5,4,2,5,NULL);
/*!40000 ALTER TABLE `oacs` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `object`
--

DROP TABLE IF EXISTS `object`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `object` (
  `object_id` int(11) NOT NULL AUTO_INCREMENT,
  `object_name` varchar(40) DEFAULT NULL,
  `object_class_id` int(11) NOT NULL,
  `map_id` int(11) DEFAULT NULL,
  `time` datetime DEFAULT NULL,
  `position_x` float DEFAULT NULL,
  `position_y` float DEFAULT NULL,
  `position_z` float DEFAULT NULL,
  `orientation_x` float DEFAULT NULL,
  `orientation_y` float DEFAULT NULL,
  `orientation_z` float DEFAULT NULL,
  `orientation_w` float DEFAULT NULL,
  `HH_object_ID` int(10) unsigned DEFAULT NULL,
  `object_shape` int(11) NOT NULL DEFAULT '1',
  `object_color` char(6) NOT NULL DEFAULT 'FF0000',
  `object_width` float NOT NULL DEFAULT '0.02',
  `object_height` float NOT NULL DEFAULT '0.02',
  PRIMARY KEY (`object_id`),
  KEY `fk_map` (`map_id`),
  CONSTRAINT `fk_map` FOREIGN KEY (`map_id`) REFERENCES `map` (`map_id`) ON DELETE NO ACTION ON UPDATE NO ACTION
) ENGINE=InnoDB AUTO_INCREMENT=12 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `object`
--

LOCK TABLES `object` WRITE;
/*!40000 ALTER TABLE `object` DISABLE KEYS */;
INSERT INTO `object` VALUES (1,'shell',2,1,'2011-08-22 14:27:53',1,3,3,0,0,0,1,NULL,1,'A79977',0.9,0.2),(2,'table1',3,1,'2011-08-22 14:36:10',-2,0,3,2,1,3.5,1,NULL,1,'ccaa88',0.8,0.8),(3,'table2',1,1,'2011-08-22 16:01:01',0.7,-1,3,2,1,3.5,1,NULL,2,'00B9B9',1.5,1),(4,'milk_box1',5,1,'2011-07-27 11:10:00',-2,1.5,3,0,0,0,0,NULL,1,'0099EE',0.05,0.2),(5,'milk_box2',4,1,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,1,'EEDD00',0.05,0.1),(6,'table3',3,2,'0000-00-00 00:00:00',2,3,5,1,2,3,4,1,1,'FF0000',0.02,0.02),(7,'table3',3,NULL,NULL,2,3,5,1,2,3,4,1,1,'FF0000',0.02,0.02),(8,'table3',3,NULL,'2011-08-11 09:56:35',2,3,5,1,2,3,4,1,1,'FF0000',0.02,0.02),(9,'table4',1,NULL,'2011-08-22 15:40:30',8,2,3,2,1,3.5,1,0,1,'FF0000',0.02,0.02),(10,'shell1',1,NULL,'2011-08-22 15:49:11',8,2,3,2,1,3.5,1,0,1,'FFFFFF',0.8,0.1),(11,'shell2',1,NULL,'2011-08-22 16:29:26',8,2,3,2,1,3.5,1,0,1,'FF0000',0.02,0.02);
/*!40000 ALTER TABLE `object` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `semantic`
--

DROP TABLE IF EXISTS `semantic`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `semantic` (
  `semantic_id` int(11) NOT NULL AUTO_INCREMENT,
  `parent_object_id` int(11) DEFAULT NULL,
  `child_object_id` int(11) DEFAULT NULL,
  `relation` varchar(45) DEFAULT NULL,
  `time` datetime DEFAULT NULL,
  PRIMARY KEY (`semantic_id`),
  KEY `fk_parent` (`parent_object_id`),
  KEY `fk_child` (`child_object_id`),
  CONSTRAINT `fk_child` FOREIGN KEY (`child_object_id`) REFERENCES `object` (`object_id`) ON DELETE NO ACTION ON UPDATE NO ACTION,
  CONSTRAINT `fk_parent` FOREIGN KEY (`parent_object_id`) REFERENCES `object` (`object_id`) ON DELETE NO ACTION ON UPDATE NO ACTION
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `semantic`
--

LOCK TABLES `semantic` WRITE;
/*!40000 ALTER TABLE `semantic` DISABLE KEYS */;
INSERT INTO `semantic` VALUES (1,1,5,'on','2011-07-27 11:07:05'),(2,2,4,'on','2011-07-27 10:35:05');
/*!40000 ALTER TABLE `semantic` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `symbolic`
--

DROP TABLE IF EXISTS `symbolic`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `symbolic` (
  `symbolic_id` int(11) NOT NULL AUTO_INCREMENT,
  `symbolic_name` varchar(45) NOT NULL,
  `Description` tinytext,
  PRIMARY KEY (`symbolic_id`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `symbolic`
--

LOCK TABLES `symbolic` WRITE;
/*!40000 ALTER TABLE `symbolic` DISABLE KEYS */;
INSERT INTO `symbolic` VALUES (1,'Near','Close to the target'),(2,'Pre_scan','Pre scanning position'),(3,'Pre_grasp','Pre grasping position'),(4,'Top_grasp','Grasp from top'),(5,'Side_grasp','Grasp from side');
/*!40000 ALTER TABLE `symbolic` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2011-09-29  9:45:14